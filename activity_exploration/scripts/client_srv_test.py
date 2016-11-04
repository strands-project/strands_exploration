#!/usr/bin/env python

import time
import rospy
import datetime
from strands_exploration_msgs.srv import GetExplorationTasks


class ActivityClientSrv(object):

    def __init__(self, start_time, end_time):
        self.service = rospy.ServiceProxy(
            "/exploration_services/activity_exp_srv",
            GetExplorationTasks
        )
        self.service.wait_for_service()
        result = self.service(start_time, end_time)
        rospy.loginfo(result)


if __name__ == "__main__":
    rospy.init_node("simple_activity_exploration_client")
    start_time = raw_input("Start time 'year month day hour minute':")
    start_time = start_time.split(" ")
    start_time = datetime.datetime(
        int(start_time[0]), int(start_time[1]), int(start_time[2]),
        int(start_time[3]), int(start_time[4])
    )
    start_time = rospy.Time(time.mktime(start_time.timetuple()))
    end_time = raw_input("End time 'year month day hour minute':")
    end_time = end_time.split(" ")
    end_time = datetime.datetime(
        int(end_time[0]), int(end_time[1]), int(end_time[2]),
        int(end_time[3]), int(end_time[4])
    )
    end_time = rospy.Time(time.mktime(end_time.timetuple()))
    ActivityClientSrv(start_time, end_time)
