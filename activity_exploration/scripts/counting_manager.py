#!/usr/bin/env python


import rospy
import random
from std_msgs.msg import String
from region_observation.util import is_intersected
from strands_navigation_msgs.msg import TopologicalMap
from strands_exploration_msgs.srv import GetExplorationTasks
from region_observation.util import robot_view_cone, get_soma_info
from strands_exploration_msgs.srv import GetExplorationTasksResponse
from periodic_poisson_processes.poisson_wrapper import PoissonWrapper
from periodic_poisson_processes.people_poisson import PoissonProcessesPeople


class PeopleCountingManager(object):

    def __init__(self, name):
        soma_config = rospy.get_param("~soma_config", "activity_exploration")
        time_window = rospy.get_param("~time_window", 10)
        time_increment = rospy.get_param("~time_increment", 1)
        periodic_cycle = rospy.get_param("~periodic_cycle", 1440)
        self.poisson_proc = PoissonProcessesPeople(
            soma_config, time_window, time_increment, periodic_cycle
        )
        self.poisson_proc.load_from_db()
        self.poisson_consent = PoissonWrapper(
            rospy.get_param("~consent_topic", "/skeleton_data/consent_ret"),
            String, "data", "nothing", time_window*3, time_increment, periodic_cycle
        )
        rospy.sleep(0.1)
        self.topo_map = None
        self.region_wps = self._get_waypoints(soma_config)
        rospy.loginfo(
            "Region ids and their nearest waypoints: %s" % str(self.region_wps)
        )
        rospy.sleep(0.1)
        self.service = rospy.Service(
            '/exploration_services/activity_exp_srv',
            GetExplorationTasks, self._srv_cb
        )
        rospy.loginfo("/exploration_services/activity_exp_srv service is ready...")
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
                "Delta start and end time is small, widening the time window..."
            )
            msg.end_time = msg.start_time + rospy.Duration(time_window*60)
        rates = self.poisson_proc.retrieve_from_to(
            msg.start_time, msg.end_time
        )
        visit_plan = list()
        for roi, poisson in rates.iteritems():
            total_rate = sum(poisson.values())
            visit_plan.append((total_rate, roi))
        visit_plan = sorted(visit_plan, key=lambda i: i[0], reverse=True)
        visit_plan = self._check_visit_plan(
            msg.start_time, msg.end_time, visit_plan
        )
        suggested_wps = list()
        suggested_score = list()
        for i in visit_plan:
            if i[1] in self.region_wps:
                if self.region_wps[i[1]] in suggested_wps:
                    suggested_score[
                        suggested_wps.index(self.region_wps[i[1]])
                    ] += i[0]
                else:
                    suggested_wps.append(self.region_wps[i[1]])
                    suggested_score.append(i[0])
            else:
                rospy.loginfo(
                    "No waypoint covers region %s, removing region..." % i[1]
                )
        task = GetExplorationTasksResponse(
            suggested_wps[:5], suggested_score[:5]
        )
        task = self._check_consent(msg, task)
        return task

    def _check_visit_plan(self, start_time, end_time, visit_plan):
        if random.random() > 0.7:
            rospy.loginfo("Changing WayPoints to visit unobserved places...")
            scales = self.poisson_proc.retrieve_from_to(
                start_time, end_time, True
            )
            scale_plan = list()
            for roi, scale in scales.iteritems():
                total_scale = sum(scale.values())
                scale_plan.append((total_scale, roi))
            scale_plan = sorted(scale_plan, key=lambda i: i[0])
            new_visit_plan = list()
            for i in scale_plan:
                for j in visit_plan:
                    if i[1] == j[1]:
                        new_visit_plan.append(j)
                        break
            visit_plan = new_visit_plan
        return visit_plan

    def _check_consent(self, msg, task):
        rates_consent = self.poisson_consent.retrieve_from_to(
            msg.start_time, msg.end_time
        )
        if sum(rates_consent.values()) > rospy.get_param("~consent_rate", 1.5):
            rospy.loginfo("Waypoint's order: %s is shuffled" % str(task.task_definition))
            random.shuffle(task.task_definition)
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
        rospy.loginfo("Getting information from /topological_map...")
        while self.topo_map is None:
            rospy.sleep(0.1)
        topo_sub.unregister()

        for wp in self.topo_map.nodes:
            wp_sight, _ = robot_view_cone(wp.pose)
            intersected_rois = list()
            intersected_regions = list()
            for roi, region in regions.iteritems():
                if is_intersected(wp_sight, region):
                    intersected_regions.append(region)
                    intersected_rois.append(roi)
            for ind, region in enumerate(intersected_regions):
                area = wp_sight.intersection(region).area
                roi = intersected_rois[ind]
                if roi in region_wps:
                    _, area1 = region_wps[roi]
                    if area > area1:
                        region_wps.update({roi: (wp.name, area)})
                else:
                    region_wps.update({roi: (wp.name, area)})
        return {
            roi: tupleoftwo[0] for roi, tupleoftwo in region_wps.iteritems()
        }


if __name__ == '__main__':
    rospy.init_node("people_count_manager")
    pcm = PeopleCountingManager(rospy.get_name())
    pcm.spin()
