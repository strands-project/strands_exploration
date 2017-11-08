#!/usr/bin/env python

import time
import datetime

import rospy
from mongodb_store.message_store import MessageStoreProxy
from exploration_bid_manager.exploration_bidder import ExplorationBidder

from popp_exploration.msg import ExplorationChoice

from multi_detect_temporal_patterns.srv import BestTimeEstimateSrv
from multi_detect_temporal_patterns.srv import BestTimeEstimateSrvResponse


class BiddingManager(object):

    def __init__(
        self, start_time=(8, 0), end_time=(22, 0),
        exploration_interval=rospy.Duration(1800), minimum_bid=500
    ):
        rospy.loginfo("Initiating bid planning...")
        # hand-allocated budget (between 8am to 6pm)
        self.exploration_interval = exploration_interval
        self._start_time = datetime.time(start_time[0], start_time[1])
        self._end_time = datetime.time(end_time[0], end_time[1])
        self.bidding_plan = list()
        self.available_tokens = 0
        self.minimum_bid = minimum_bid
        tmp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        self._bidding_date = datetime.datetime(
            tmp.year, tmp.month, tmp.day, 0, 0
        )
        self.bidder = ExplorationBidder()
        self.soma_config = rospy.get_param(
            "~soma_config", "poisson_activity"
        )
        self._db = MessageStoreProxy(collection="popp_exploration_log")
        # all services to counters
        popp_srv_name = "/multi_sensor_counting_process/best_time_estimate"
        rospy.loginfo("Connecting to %s service..." % popp_srv_name)
        self._popp_srv = rospy.ServiceProxy(popp_srv_name, BestTimeEstimateSrv)
        self._popp_srv.wait_for_service()

    def get_popp_estimate(self, start, end, size):
        result = None
        try:
            result = self._popp_srv(start, end, size, True)
        except NameError:
            result = BestTimeEstimateSrvResponse(list(), list(), list())
        return result

    def calculate_bidding_plan(self, recommended_regions=[], time_visited_regions=[]):
        current_time = datetime.datetime.fromtimestamp(
            (rospy.Time.now().secs / 86400) * 86400
        )
        available_tokens = self.bidder.available_tokens - self.bidder.currently_bid_tokens
        if current_time-self._bidding_date >= datetime.timedelta(days=1):
            rospy.loginfo(
                "Calculating bidding plan for %s..." % str(
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
            self._calculate_bidding_plan(
                start, end, available_tokens, recommended_regions, time_visited_regions
            )
            self._bidding_date = current_time
        elif available_tokens:
            start = datetime.datetime.fromtimestamp(
                (rospy.Time.now().secs / 60) * 60
            )
            rospy.loginfo(
                "Calculating bidding plan with %d avaible bid from %s..." % (
                    available_tokens, str(start)
                )
            )
            end = datetime.datetime(
                start.year, start.month, start.day,
                self._end_time.hour, self._end_time.minute
            )
            start = rospy.Time(time.mktime(start.timetuple()))
            end = rospy.Time(time.mktime(end.timetuple()))
            self._calculate_bidding_plan(
                start, end, available_tokens, recommended_regions, time_visited_regions
            )
        else:
            rospy.loginfo("No available bid, skipping process..")

    def _calculate_bidding_plan(
        self, start, end, available_tokens, recommended_regions=[], time_visited_regions=[]
    ):
        rospy.loginfo("Available tokens: %d" % available_tokens)
        estimates = list()
        while (start + self.exploration_interval) <= end:
            estimate = self.get_popp_estimate(start, start + self.exploration_interval, 5)
            if recommended_regions != list():
                estimate = [
                    (
                        estimate.times[j], i, estimate.estimates[j]
                    ) for j, i in enumerate(estimate.region_ids) if i in recommended_regions
                ]
            if not len(estimate):
                rospy.logwarn(
                    "These regions %s are not covered by any waypoint" % str(zip(*estimate)[1])
                )
            if time_visited_regions != list():
                estimate = [i for i in estimate if (i[0], i[1]) not in time_visited_regions]
            if not len(estimate):
                rospy.loginfo("No extra exploration needed...")
            if len(estimate):
                estimates.append(estimate[0])
            start = start + self.exploration_interval
        self._merge_plan_with_tokens(estimates, available_tokens)
        if len(self.bidding_plan):
            # store options to db
            msg = ExplorationChoice()
            msg.soma_config = self.soma_config
            for i in estimates:
                if i[0] in zip(*self.bidding_plan)[0]:
                    msg.start_times.append(i[0])
                    msg.region_ids.append(i[1])
                    msg.estimates.append(i[2])
                    msg.bid.append(
                        zip(*self.bidding_plan)[2][
                            zip(*self.bidding_plan)[0].index(i[0])
                        ]
                    )
            self._db.insert(msg)
        self.available_tokens = available_tokens
        rospy.loginfo("Feasible bidding plan: %s" % str(self.bidding_plan))

    def _merge_plan_with_tokens(self, estimates, available_tokens):
        if len(estimates):
            # normalize estimate
            norm = sum(zip(*estimates)[2])
            if norm > 0.0:
                proposed_bidding_plan = [
                    (
                        i[0], i[1], i[2]
                    ) for i in estimates if int(
                        (i[2]/norm)*available_tokens
                    ) >= self.minimum_bid
                ]
                if len(proposed_bidding_plan):
                    norm = sum(zip(*proposed_bidding_plan)[2])
                else:
                    rospy.logwarn("Allocated budget does not meet the minimum bid required.")
            elif int(
                (1 / float(len(estimates))) * available_tokens
            ) >= self.minimum_bid:
                norm = float(len(estimates))
                proposed_bidding_plan = [(i[0], i[1], 1) for i in estimates]
            else:
                estimates = estimates[:len(estimates)/2]
                norm = float(len(estimates))
                proposed_bidding_plan = [(i[0], i[1], 1) for i in estimates]
            self.bidding_plan = [
                (
                    i[0], i[1], int((i[2]/norm)*available_tokens)
                ) for i in proposed_bidding_plan
            ]
        else:
            self.bidding_plan = list()
