#!/usr/bin/env python


import rospy
import random
from std_msgs.msg import String
from strands_navigation_msgs.msg import TopologicalMap
from region_observation.util import robot_view_cone, get_soma_info
from periodic_poisson_processes.poisson_wrapper import PoissonWrapper
from periodic_poisson_processes.people_poisson import PoissonProcessesPeople
from region_observation.util import is_intersected, get_largest_intersected_regions
from strands_exploration_msgs.srv import GetExplorationTasks, GetExplorationTasksResponse


class PeopleCountingManager(object):

    def __init__(self, name):
        soma_config = rospy.get_param("~soma_config", "activity_exploration")
        time_window = rospy.get_param("~time_window", 10)
        time_increment = rospy.get_param("~time_increment", 1)
        periodic_cycle = rospy.get_param("~periodic_cycle", 10080)
        self.poisson_proc = PoissonProcessesPeople(
            soma_config, time_window, time_increment, periodic_cycle
        )
        self.poisson_proc.load_from_db()
        self.poisson_consent = PoissonWrapper(
            rospy.get_param("~consent_topic", "/skeleton_data/consent_ret"), String,
            "data", "nothing", window=time_window*3, increment=time_increment,
            periodic_cycle=periodic_cycle/7
        )
        rospy.sleep(0.1)
        self.topo_map = None
        self.region_wps = self._get_waypoints(soma_config)
        rospy.loginfo("Region ids and their nearest waypoints: %s" % str(self.region_wps))
        rospy.sleep(0.1)
        rospy.loginfo("Create a service %s/get_waypoints..." % name)
        self.service = rospy.Service(name+'/get_waypoints', GetExplorationTasks, self._srv_cb)
        rospy.sleep(0.1)

    def spin(self):
        self.poisson_proc.continuous_update()
        rospy.spin()

    def _srv_cb(self, msg):
        rospy.loginfo(
            "Got a request to find waypoints to visit between %d and %d"
            % (msg.start_time.secs, msg.end_time.secs)
        )
        time_window = rospy.get_param("~time_window", 10)
        if (msg.end_time - msg.start_time).secs < time_window*60:
            rospy.logwarn(
                "Time window between start and end time is too small, widening the time window..."
            )
            msg.end_time = msg.start_time + rospy.Duration(time_window*60)
        rates = self.poisson_proc.retrieve_from_to(
            msg.start_time, msg.end_time
        )
        result = list()
        for roi, poisson in rates.iteritems():
            total_rate = sum(poisson.values())
            result.append((total_rate, roi))
        result = sorted(result, key=lambda i: i[0], reverse=True)
        total = float(sum([i[0] for i in result]))
        try:
            task = GetExplorationTasksResponse(
                map(lambda i: self.region_wps[i], [i[1] for i in result])[:5],
                map(lambda i: i/total, [i[0] for i in result])[:5]
            )
        except:
            temp = map(lambda i: self.region_wps[i], [i[1] for i in result])[:5]
            rospy.logwarn("Ups...no suggestion for waypoints is found at the moment")
            task = GetExplorationTasksResponse(
                temp,
                [1/float(len(temp)) for i in range(len(temp))]
            )
        task = self._check_consent(msg, task)
        print task
        return task

    def _check_consent(self, msg, task):
        rates_consent = self.poisson_consent.retrieve_from_to(
            msg.start_time, msg.end_time
        )
        if sum(rates_consent.values()) > rospy.get_param("~consent_rate", 1.5):
            rospy.loginfo("Waypoint's order: %s" % str(task.task_definition))
            rospy.logwarn("Consent rate shows too many rejections with this configuration...")
            rospy.logwarn("Shuffling suggested waypoints...")
            random.shuffle(task.task_definition)
            rospy.loginfo("New waypoint's order: %s" % str(task.task_definition))
        return task

    def _topo_map_cb(self, topo_map):
        self.topo_map = topo_map

    def _get_waypoints(self, soma_config):
        region_wps = dict()
        # get regions information
        regions, _ = get_soma_info(soma_config)
        # get waypoint information
        topo_sub = rospy.Subscriber(
            "/topological_map", TopologicalMap, self._topo_map_cb, None, 10
        )
        while self.topo_map is None:
            rospy.sleep(0.1)
            rospy.logwarn("Trying to get information from /topological_map...")
        topo_sub.unregister()

        for wp in self.topo_map.nodes:
            wp_sight, _ = robot_view_cone(wp.pose)
            intersected_rois = list()
            intersected_regions = list()
            for roi, region in regions.iteritems():
                if is_intersected(wp_sight, region):
                    intersected_regions.append(region)
                    intersected_rois.append(roi)
            if len(intersected_regions) > 1:
                rospy.logwarn(
                    "There are two or more regions covered by the sight of the robot in %s" % wp.name
                )
                rospy.loginfo("Trying to get the largest intersected area between robot's sight and regions")
                region = get_largest_intersected_regions(wp_sight, intersected_regions)
                roi = intersected_rois[intersected_regions.index(region)]
            elif len(intersected_regions) == 1:
                region = intersected_regions[0]
                roi = intersected_rois[0]
            else:
                rospy.logwarn("No region is covered by %s!" % wp.name)
                continue
            area = wp_sight.intersection(region).area
            if roi in region_wps:
                _, area1 = region_wps[roi]
                if area > area1:
                    region_wps.update({roi: (wp.name, area)})
            else:
                region_wps.update({roi: (wp.name, area)})
        return {roi: tupleoftwo[0] for roi, tupleoftwo in region_wps.iteritems()}


if __name__ == '__main__':
    rospy.init_node("people_count_manager")
    pcm = PeopleCountingManager(rospy.get_name())
    pcm.spin()
