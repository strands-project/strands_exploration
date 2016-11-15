#!/usr/bin/env python

import copy
import time
import rospy
import datetime
import numpy as np
from exploration_bid_manager.exploration_bidder import ExplorationBidder
from scene_temporal_patterns.srv import SceneBestTimeEstimateSrv
from scene_temporal_patterns.srv import SceneBestTimeEstimateSrvResponse
from people_temporal_patterns.srv import PeopleBestTimeEstimateSrv
from people_temporal_patterns.srv import PeopleBestTimeEstimateSrvResponse
from activity_temporal_patterns.srv import ActivityBestTimeEstimateSrv
from activity_temporal_patterns.srv import ActivityBestTimeEstimateSrvResponse


class BudgetControl(object):

    def __init__(
        self, start_time=(8, 0), end_time=(18, 0), max_visit=20,
        update_interval=rospy.Duration(86400)
    ):
        rospy.loginfo("Initiating budgetting control...")
        # hand-allocated budget (between 8am to 6pm)
        self._max_visit = max_visit
        self._update_interval = update_interval
        self._start_time = datetime.time(start_time[0], start_time[1])
        self._end_time = datetime.time(end_time[0], end_time[1])
        self.budget_alloc = None
        self.total_budget = None
        tmp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        self._update_budget_date = datetime.datetime(
            tmp.year, tmp.month, tmp.day, 0, 0
        )
        self.bidder = ExplorationBidder()
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
        # calling budget allocation periodically
        self._get_budget_alloc(None)
        rospy.Timer(update_interval, self._get_budget_alloc)

    def get_people_estimate(self, start, end):
        result = None
        try:
            result = self._people_srv(start, end, self._max_visit, True)
        except NameError:
            result = PeopleBestTimeEstimateSrvResponse(list(), list(), list())
        result.estimates = [i/np.linalg.norm(i) for i in result.estimates]
        return result

    def get_scene_estimate(self, start, end):
        result = None
        try:
            result = self._scene_srv(start, end, self._max_visit, True)
        except NameError:
            result = SceneBestTimeEstimateSrvResponse(list(), list(), list())
        result.estimates = [i/np.linalg.norm(i) for i in result.estimates]
        return result

    def get_activity_estimate(self, start, end):
        result = None
        try:
            result = self._act_srv(start, end, self._max_visit, True)
        except NameError:
            result = ActivityBestTimeEstimateSrvResponse(list(), list(), list())
        result.estimates = [i/np.linalg.norm(i) for i in result.estimates]
        return result

    def get_norm_estimate(self, start, end):
        result = list()  # (times, regions, estimates)
        # normalized estimate
        scene = self.get_scene_estimate(start, end)
        people = self.get_people_estimate(start, end)
        activity = self.get_activity_estimate(start, end)
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
                result.append((xtime, act_roi, scene_est+people_est+act_est))
            elif scene_roi == act_roi != "":
                result.append((xtime, act_roi, scene_est+act_est))
                result.append((xtime, people_roi, people_est))
            elif scene_roi == people_roi != "":
                result.append((xtime, scene_roi, scene_est+people_est))
                result.append((xtime, act_roi, act_est))
            elif people_roi == act_roi != "":
                result.append((xtime, act_roi, act_est+people_est))
                result.append((xtime, scene_roi, scene_est))
            else:
                result.append((xtime, act_roi, act_est))
                result.append((xtime, scene_roi, scene_est))
                result.append((xtime, people_roi, people_est))
        result = sorted(result, key=lambda i: i[2], reverse=True)
        result = result[:self._max_visit]
        # normalize estimate
        norm = np.linalg.norm(zip(*result)[2])
        result = [
            (i[0], i[1], i[2]/norm) for i in result
        ]
        return result

    def _get_budget_alloc(self, event):
        tmp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        current_time = datetime.datetime(tmp.year, tmp.month, tmp.day, 0, 0)
        if self.budget_alloc is None or (
            self._update_budget_date-current_time
        ) >= datetime.timedelta(days=1):
            total_budget = self.bidder.available_tokens - self.bidder.currently_bid_tokens
            rospy.loginfo("Current budget: %d" % total_budget)
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
            estimate = self.get_norm_estimate(start, end)
            self.budget_alloc = [
                (i[0], i[1], i[2]*total_budget) for i in estimate
            ]
            rospy.loginfo("Budget allocation: %s" % str(self.budget_alloc))
            self.total_budget = total_budget
            self._update_budget_date = current_time
