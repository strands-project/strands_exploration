#!/usr/bin/env python

import copy
import time
import math
import rospy
import argparse
import datetime
from human_trajectory.msg import Trajectories
from region_observation.observation_proxy import RegionObservationProxy
from region_observation.util import create_line_string, is_intersected, get_soma_info
from periodic_poisson_processes.poisson_processes import PeriodicPoissonProcesses


class PoissonProcessesPeople(object):

    def __init__(
        self, config, window=10, increment=1,
        periodic_cycle=10080, coll="poisson_processes"
    ):
        rospy.loginfo("Initializing people counting...")
        temp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        temp = datetime.datetime(
            temp.year, temp.month, temp.day, temp.hour, temp.minute, 0
        )
        self._start_time = rospy.Time(time.mktime(temp.timetuple()))
        self._acquired = False
        # trajectories subscriber
        self.trajectories = list()
        rospy.Subscriber(
            rospy.get_param("~trajectory_topic", "/people_trajectory/trajectories/complete"),
            Trajectories, self._pt_cb, None, 10
        )
        # get soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, self.config)
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        self.time_window = window
        self.time_increment = increment
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        self.process = {
            roi: PeriodicPoissonProcesses(window, increment, periodic_cycle) for roi in self.regions.keys()
        }

    def retrieve_from_to(self, start_time, end_time):
        result = dict()
        for roi, poisson in self.process.iteritems():
            result.update(
                {roi: poisson.retrieve(start_time, end_time)}
            )
        return result

    def load_from_db(self):
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "people"
            }
            self.process[roi].retrieve_from_mongo(meta)

    def store_to_db(self):
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "people"
            }
            self.process[roi].store_to_mongo(meta)

    def _pt_cb(self, msg):
        if len(msg.trajectories) == 0:
            return
        while self._acquired:
            rospy.sleep(0.01)
        self._acquired = True
        self.trajectories.extend([trajectory for trajectory in msg.trajectories])
        self._acquired = False

    def continuous_update(self):
        while not rospy.is_shutdown():
            delta = (rospy.Time.now() - self._start_time)
            rospy.loginfo("%d seconds" % delta.secs)
            if delta > rospy.Duration(self.time_window*60):
                rospy.loginfo("Updating poisson processes for each region...")
                self.update()
            rospy.sleep(1)

    def update(self):
        region_observations = self.obs_proxy.load_msg(
            self._start_time, rospy.Time.now(), minute_increment=self.time_increment
        )
        temp = copy.deepcopy(self.trajectories)
        rospy.loginfo("Total trajectories counted so far is %d." % len(temp))

        traj_inds = list()
        self._start_time = self._start_time + rospy.Duration(self.time_increment*60)
        for observation in region_observations:
            rospy.loginfo(
                "Observation in region %s was for %d duration from %d to %d." % (
                    observation.region_id, observation.duration.secs, observation.start_from.secs, observation.until.secs
                )
            )
            count = 0
            for ind, trajectory in enumerate(temp):
                points = [
                    [
                        pose.pose.position.x, pose.pose.position.y
                    ] for pose in trajectory.trajectory
                ]
                points = create_line_string(points)
                if is_intersected(self.regions[observation.region_id], points):
                    if trajectory.end_time >= observation.start_from:
                        count += 1
                if trajectory.end_time > self._start_time and ind not in traj_inds:
                    traj_inds.append(ind)
            if count > 0 or observation.duration.secs >= 59:
                count = self._extrapolate_count(observation.duration, count)
                self.process[observation.region_id].update(observation.start_from, count)
                # save the observation for that time
                self._store(observation.region_id, observation.start_from)
        # clear observed region to get new info, remove trajectories that
        # have been updated
        n = len(temp)
        temp = [temp[i] for i in traj_inds]
        while self._acquired:
            rospy.sleep(0.01)
        self._acquired = True
        self.trajectories = temp + self.trajectories[n:]
        self._acquired = False

    def _store(self, roi, start_time):
        self.process[roi]._store(
            start_time,
            {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "people"
            }
        )

    def _extrapolate_count(self, duration, count):
        """ extrapolate the number of trajectories with specific upper_threshold.
            upper_threshold is to ceil how long the robot was in an area
            for one minute interval, if the robot was there for less than 20
            seconds, then it will be boosted to 20 seconds.
        """
        upper_threshold_duration = rospy.Duration(0, 0)
        while duration > upper_threshold_duration:
            upper_threshold_duration += rospy.Duration(
                self.time_increment * 20, 0
            )

        multiplier_estimator = 3600 / float(
            (60 / self.time_increment) * upper_threshold_duration.secs
        )
        rospy.loginfo("Extrapolate count %d by %.2f" % (count, multiplier_estimator))
        return math.ceil(multiplier_estimator * count)


if __name__ == '__main__':
    rospy.init_node("poisson_processes_people")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument('soma_config', help="Soma configuration")
    parser.add_argument(
        "-w", dest="time_window", default="10",
        help="Fixed time window interval (in minute) for each Poisson distribution. Default is 10 minute."
    )
    parser.add_argument(
        "-m", dest="minute_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute."
    )
    parser.add_argument(
        "-p", dest="periodic_cycle", default="10080",
        help="Incremental time (in minute). Default is one week (10080 minutes)."
    )
    args = parser.parse_args()

    ppp = PoissonProcessesPeople(
        args.soma_config, int(args.time_window), int(args.minute_increment),
        int(args.periodic_cycle)
    )
    ppp.load_from_db()
    ppp.continuous_update()
    rospy.spin()
    # ppp.store_to_db()
    ppp.retrieve_from_to(
        rospy.Time(1462108560), rospy.Time(1462108980)
    )
