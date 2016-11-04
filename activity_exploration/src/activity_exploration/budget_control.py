#!/usr/bin/env python


import rospy
import datetime
import threading
from people_temporal_patterns.srv import PeopleEstimateSrv
from exploration_bid_manager.exploration_bidder import ExplorationBidder


class BudgetControl(object):

    def __init__(
        self, start_time=(8, 0), end_time=(18, 0), time_alloc_multi=[1, 2, 2, 1]
    ):
        rospy.loginfo("Initiating budgetting control...")
        # hand-allocated budget (between 8am to 6pm)
        self._time_alloc_multi = time_alloc_multi
        self._start_time = datetime.time(start_time[0], start_time[1])
        self._end_time = datetime.time(end_time[0], end_time[1])
        self.budget_alloc = None
        self.total_budget = None
        temp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        self._update_budget_date = datetime.datetime(
            temp.year, temp.month, temp.day, 0, 0
        )
        self.bidder = ExplorationBidder()
        # rospy.loginfo("Connect to /exploration_services/budget_info service...")
        # self._exp_info_srv = rospy.ServiceProxy(
        #     "/exploration_services/budget_info", GetBudgetInfo
        # )
        # self._exp_info_srv.wait_for_service()
        people_srv_name = rospy.get_param(
            "~people_srv", "/people_counter/people_estimate"
        )
        rospy.loginfo("Connecting to %s service..." % people_srv_name)
        self._people_srv = rospy.ServiceProxy(people_srv_name, PeopleEstimateSrv)
        self._people_srv.wait_for_service()
        rospy.sleep(0.1)
        self._time_alloc = self._get_time_alloc(self._start_time, self._end_time)
        self._thread = threading.Thread(target=self._get_budget_alloc)
        self._thread.start()

    def _get_time_alloc(self, start_time, end_time):
        budget_cats = len(self._time_alloc_multi)
        start = rospy.Time.now()
        start = datetime.datetime.fromtimestamp(start.secs)
        start = datetime.datetime(
            start.year, start.month, start.day,
            start_time.hour, start_time.minute
        )
        end = datetime.datetime(
            start.year, start.month, start.day,
            end_time.hour, end_time.minute
        )
        dur = int((end-start).total_seconds())
        time_alloc = list()
        for i in range(budget_cats):
            added_hour = ((i * dur) / budget_cats) / 3600
            added_min = (((i * dur) / budget_cats) % 3600) / 60
            start = datetime.time(start_time.hour + added_hour, start_time.minute + added_min)
            added_hour = (((i+1) * dur) / budget_cats) / 3600
            added_min = ((((i+1) * dur) / budget_cats) % 3600) / 60
            if added_min == 0:
                added_hour = (added_hour - 1) % 24
            added_min = (added_min - 1) % 60
            end = datetime.time(start_time.hour + added_hour, start_time.minute + added_min, 59)
            time_alloc.append((start, end, self._time_alloc_multi[i]))
        return time_alloc

    def _get_budget_alloc(self):
        # assuming arg for BudgetInfo is string of the name of exploration
        # returning budge left
        while not rospy.is_shutdown():
            temp = datetime.datetime.fromtimestamp(rospy.Time.now())
            current_time = datetime.datetime(
                temp.year, temp.month, temp.day, 0, 0
            )
            if self.budget_alloc is None or (
                self._update_budget_date-current_time
            ) >= datetime.timedelta(days=1):
                # budget = self._exp_info_srv("activity_exploration")
                budget = self.bidder.available_tokens - self.bidder.currently_bid_tokens
                total_budget = budget.budget
                total_mult = float(sum([i[2] for i in self._time_alloc]))
                budget_alloc = list()
                for i in self._time_alloc:
                    budget_alloc.append((i[0], i[1], int(i[2] / total_mult * total_budget)))
                self.budget_alloc = budget_alloc
                self.total_budget = total_budget
                self._update_budget_date = current_time
            rospy.sleep(300)

    def get_allocated_budget(self, start_time, end_time):
        start = datetime.datetime.fromtimestamp(start_time.secs)
        start = datetime.time(start.hour, start.minute)
        budget = self.bidder.available_tokens - self.bidder.currently_bid_tokens
        total_budget = budget.budget
        non_allocated = 0
        end_budget_time = None
        for i in self.budget_alloc:
            if start < i[0]:
                non_allocated += i[2]
            elif start <= i[1]:
                end_budget_time = i[1]
        assert end_budget_time is not None, "end_budget_time is not assigned somehow"
        allocated_budget = total_budget - non_allocated
        # make allocated budget proportional to the importance of exploration
        people_estimates = self._people_srv(start_time, end_budget_time, True, False)
        total_estimate = sum(sorted(people_estimates.estimates, reverse=True)[:3])
        people_estimates = self._people_srv(start_time, end_time, True, False)
        portion_estimate = sum(sorted(people_estimates.estimates, reverse=True)[:3])
        if total_estimate > 0:
            allocated_budget = min([int(portion_estimate/total_estimate*2), allocated_budget])
        return allocated_budget
