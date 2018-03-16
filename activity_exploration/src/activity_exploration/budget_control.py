#!/usr/bin/env python

import copy
import time
import rospy
import datetime
import numpy as np
from activity_exploration.msg import ExplorationChoice
from mongodb_store.message_store import MessageStoreProxy
from exploration_bid_manager.exploration_bidder import ExplorationBidder
from scene_temporal_patterns.srv import SceneBestTimeEstimateSrv
from scene_temporal_patterns.srv import SceneBestTimeEstimateSrvResponse
from people_temporal_patterns.srv import PeopleBestTimeEstimateSrv
from people_temporal_patterns.srv import PeopleBestTimeEstimateSrvResponse
from activity_temporal_patterns.srv import ActivityBestTimeEstimateSrv
from activity_temporal_patterns.srv import ActivityBestTimeEstimateSrvResponse


class BudgetControl(object):

    def __init__(
        self, start_time=(9, 0), end_time=(17, 0),
        observe_interval=rospy.Duration(1800), minimal_required_budget=500
    ):
        rospy.loginfo("Initiating budgetting control...")
        # hand-allocated budget (between 8am to 6pm)
        self.observe_interval = observe_interval
        self._start_time = datetime.time(start_time[0], start_time[1])
        self._end_time = datetime.time(end_time[0], end_time[1])
        self.budget_alloc = list()
        self.available_budget = 0
        self.minimal_required_budget = minimal_required_budget
        tmp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        self._update_budget_date = datetime.datetime(
            tmp.year, tmp.month, tmp.day, 0, 0
        )
        self.bidder = ExplorationBidder()
        self.soma_config = rospy.get_param(
            "~soma_config", "activity_exploration"
        )
        self._db = MessageStoreProxy(collection="activity_exploration_log")
        # all services to counters
        people_srv_name = rospy.get_param(
            "~people_srv", "/people_counter/people_best_time_estimate"
        )
        if people_srv_name != "":
            rospy.loginfo("Connecting to %s service..." % people_srv_name)
            self._people_srv = rospy.ServiceProxy(
                people_srv_name, PeopleBestTimeEstimateSrv
            )
            self._people_srv.wait_for_service()
        act_srv_name = rospy.get_param(
            "~activity_srv", "/activity_counter/activity_best_time_estimate"
        )
        if act_srv_name != "":
            rospy.loginfo("Connecting to %s service..." % act_srv_name)
            self._act_srv = rospy.ServiceProxy(
                act_srv_name, ActivityBestTimeEstimateSrv
            )
            self._act_srv.wait_for_service()
        scene_srv_name = rospy.get_param(
            "~scene_srv", "/scene_counter/scene_best_time_estimate"
        )
        if scene_srv_name != "":
            rospy.loginfo("Connecting to %s service..." % scene_srv_name)
            self._scene_srv = rospy.ServiceProxy(
                scene_srv_name, SceneBestTimeEstimateSrv
            )
            self._scene_srv.wait_for_service()

    def get_people_estimate(self, start, end, size):
        result = None
        try:
            result = self._people_srv(start, end, size, True)
        except NameError:
            result = PeopleBestTimeEstimateSrvResponse(list(), list(), list())
        norm = np.linalg.norm(result.estimates)
        if norm > 0.0:
            result.estimates = [i/norm for i in result.estimates]
        return result

    def get_scene_estimate(self, start, end, size):
        result = None
        try:
            result = self._scene_srv(start, end, size, True)
        except NameError:
            result = SceneBestTimeEstimateSrvResponse(list(), list(), list())
        norm = np.linalg.norm(result.estimates)
        if norm > 0.0:
            result.estimates = [i/norm for i in result.estimates]
        return result

    def get_activity_estimate(self, start, end, size):
        result = None
        try:
            result = self._act_srv(start, end, size, True)
        except NameError:
            result = ActivityBestTimeEstimateSrvResponse(list(), list(), list())
        norm = np.linalg.norm(result.estimates)
        if norm > 0.0:
            result.estimates = [i/norm for i in result.estimates]
        return result

    def get_norm_estimate(self, start, end, size):
        result = list()  # (times, regions, estimates)
        # normalized estimate
        scene = self.get_scene_estimate(start, end, size)
        people = self.get_people_estimate(start, end, size)
        activity = self.get_activity_estimate(start, end, size)
        times = copy.deepcopy(scene.times + people.times + activity.times)
        for xtime in set(times):
            act_roi = ""
            act_est = .0
            if xtime in activity.times:
                act_est = activity.estimates[activity.times.index(xtime)]
                act_roi = activity.region_ids[activity.times.index(xtime)]
            people_roi = ""
            people_est = .0
            if xtime in people.times:
                people_est = people.estimates[people.times.index(xtime)]
                people_roi = people.region_ids[people.times.index(xtime)]
            scene_roi = ""
            scene_est = .0
            if xtime in scene.times:
                scene_est = scene.estimates[scene.times.index(xtime)]
                scene_roi = scene.region_ids[scene.times.index(xtime)]
            if scene_roi == act_roi == people_roi != "":
                result.append(
                    (
                        xtime, act_roi, scene_est+people_est+act_est,
                        "scene_people_activity"
                    )
                )
            elif scene_roi == act_roi != "":
                result.append(
                    (xtime, act_roi, scene_est+act_est, "scene_activity")
                )
                result.append(
                    (xtime, people_roi, people_est, "people")
                )
            elif scene_roi == people_roi != "":
                result.append(
                    (xtime, scene_roi, scene_est+people_est, "scene_people")
                )
                result.append((xtime, act_roi, act_est, "activity"))
            elif people_roi == act_roi != "":
                result.append(
                    (xtime, act_roi, act_est+people_est, "people_activity")
                )
                result.append((xtime, scene_roi, scene_est, "scene"))
            else:
                result.append((xtime, act_roi, act_est, "activity"))
                result.append((xtime, scene_roi, scene_est, "scene"))
                result.append((xtime, people_roi, people_est, "people"))
        result = sorted(result, key=lambda i: i[2], reverse=True)
        result = result[:size]
        return result

    def get_budget_alloc(self, recommended_rois=[], non_recommended_rois=[]):
        tmp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        current_time = datetime.datetime(tmp.year, tmp.month, tmp.day, 0, 0)
        total_budget = self.bidder.available_tokens - self.bidder.currently_bid_tokens
        if current_time-self._update_budget_date >= datetime.timedelta(days=1):
            rospy.loginfo(
                "Planning budget allocation for %s..." % str(
                    current_time.date()
                )
            )
            start = datetime.datetime(
                current_time.year, current_time.month, current_time.day,
                self._start_time.hour, self._start_time.minute
            )
            end = datetime.datetime(
                current_time.year, current_time.month, current_time.day,
                self._end_time.hour, self._end_time.minute
            )
            start = rospy.Time(time.mktime(start.timetuple()))
            end = rospy.Time(time.mktime(end.timetuple()))
            self._get_budget_alloc(
                start, end, total_budget, recommended_rois, non_recommended_rois
            )
            self._update_budget_date = current_time
        elif total_budget:
            tmp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
            current_time = datetime.datetime(
                tmp.year, tmp.month, tmp.day, tmp.hour, tmp.minute
            )
            rospy.loginfo(
                "Planning budget allocation for today from %s..." % str(
                    current_time
                )
            )
            start = datetime.datetime(
                current_time.year, current_time.month, current_time.day,
                current_time.hour, current_time.minute
            )
            end = datetime.datetime(
                current_time.year, current_time.month, current_time.day,
                self._end_time.hour, self._end_time.minute
            )
            start = rospy.Time(time.mktime(start.timetuple()))
            end = rospy.Time(time.mktime(end.timetuple()))
            self._get_budget_alloc(
                start, end, total_budget, recommended_rois, non_recommended_rois
            )
        else:
            rospy.loginfo("No budget available, skipping process..")

    def _calculate_budget_alloc(self, estimates, total_budget):
        if len(estimates):
            # normalize estimate
            norm = sum(zip(*estimates)[2])
            if norm > 0.0:
                proposed_budget_alloc = [
                    (
                        i[0], i[1], i[2]
                    ) for i in estimates if int(
                        (i[2]/norm)*total_budget
                    ) >= self.minimal_required_budget
                ]
                if len(proposed_budget_alloc):
                    norm = sum(zip(*proposed_budget_alloc)[2])
                else:
                    rospy.logwarn("Allocated budget does not meet the minimal required budget")
            elif int(
                (1 / float(len(estimates))) * total_budget
            ) >= self.minimal_required_budget:
                norm = float(len(estimates))
                proposed_budget_alloc = [(i[0], i[1], 1) for i in estimates]
            else:
                estimates = estimates[:len(estimates)/2]
                norm = float(len(estimates))
                proposed_budget_alloc = [(i[0], i[1], 1) for i in estimates]
            self.budget_alloc = [
                (
                    i[0], i[1], int((i[2]/norm)*total_budget)
                ) for i in proposed_budget_alloc
            ]
        else:
            self.budget_alloc = list()

    def _get_budget_alloc(
        self, start, end, total_budget, recommended_rois=[], non_recommended_rois=[]
    ):
        rospy.loginfo("Available budget: %d" % total_budget)
        norms = list()
        estimates = list()
        while (start + self.observe_interval) <= end:
            estimate = self.get_norm_estimate(
                start, start + self.observe_interval, 5
            )
            if recommended_rois != list():
                estimate = [i for i in estimate if i[1] in recommended_rois]
            if not len(estimate):
                rospy.logwarn("The recommended places are not ones in the config file")
            if non_recommended_rois != list():
                estimate = [i for i in estimate if (i[0], i[1]) not in non_recommended_rois]
            if not len(estimate):
                rospy.loginfo("No new places need to be added.")
            if len(estimate):
                estimates.append(estimate[0])
            start = start + self.observe_interval
        self._calculate_budget_alloc(estimates, total_budget)
        if len(self.budget_alloc):
            # store options to db
            msg = ExplorationChoice()
            msg.soma_config = self.soma_config
            for i in estimates:
                if i[0] in zip(*self.budget_alloc)[0]:
                    msg.start_times.append(i[0])
                    msg.region_ids.append(i[1])
                    msg.estimates.append(i[2])
                    msg.contributing_models.append(i[3])
            self._db.insert(msg)
        self.available_budget = total_budget
        rospy.loginfo("Budget allocation: %s" % str(self.budget_alloc))
