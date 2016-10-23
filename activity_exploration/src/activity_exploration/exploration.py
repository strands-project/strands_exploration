#!/usr/bin/env python


import yaml
import rospy
import roslib
import random
from std_srvs.srv import Empty

from activity_exploration.srv import ChangeMethodSrv
from activity_exploration.srv import ChangeMethodSrvResponse
from activity_exploration.budget_control import BudgetControl

from people_temporal_patterns.srv import PeopleEstimateSrv
from activity_temporal_patterns.srv import ActivityEstimateSrv

from strands_navigation_msgs.msg import TopologicalMap
from strands_exploration_msgs.srv import GetExplorationTasks
from strands_exploration_msgs.srv import GetExplorationTasksResponse

from region_observation.util import is_intersected
from region_observation.util import robot_view_cone, get_soma_info


class ActivityRecommender(object):

    def __init__(self):
        rospy.loginfo("Initiating activity exploration...")
        self.exploration_method = rospy.get_param("~exploration_method", "ubc")
        self.budget_control = BudgetControl()
        self._exp_req_dur = rospy.Duration(rospy.get_param("~exp_req_duration", 3600))
        rospy.loginfo(
            "Connecting to %s service..." % rospy.get_param(
                "~people_srv", "/people_counter/people_estimate"
            )
        )
        self.people_srv = rospy.ServiceProxy(
            rospy.get_param("~people_srv", "/people_counter/people_estimate"),
            PeopleEstimateSrv
        )
        self.people_srv.wait_for_service()
        srv_name = rospy.get_param("~people_srv", "/people_counter/people_estimate")
        srv_name = srv_name.split("/")[1]
        self.people_restart_srv = rospy.ServiceProxy("/"+srv_name+"/restart", Empty)
        self.people_restart_srv.wait_for_service()
        rospy.sleep(0.1)
        act_srv_name = rospy.get_param("~activity_srv", "")
        self.act_srv = None
        if act_srv_name != "":
            rospy.loginfo("Connecting to %s service..." % act_srv_name)
            self.act_srv = rospy.ServiceProxy(act_srv_name, ActivityEstimateSrv)
            self.act_srv.wait_for_service()
            rospy.sleep(0.1)
        self.epsilon = 0.15
        self.topo_map = None
        if rospy.get_param("~with_config_file", False):
            self.region_wps = self._get_waypoints_from_file()
        else:
            self.region_wps = self._get_waypoints(
                rospy.get_param("~soma_config", "activity_exploration")
            )

        rospy.loginfo(
            "Region ids and their nearest waypoints: %s" % str(self.region_wps)
        )
        rospy.loginfo("Connecting to /exploration_services/SOMETHING service...")
        self._exp_add_srv = rospy.ServiceProxy("/exploration_services/add", GetExplorationTasks)
        rospy.sleep(0.1)
        rospy.Service(
            '%s/change_method_srv' % rospy.get_name(),
            ChangeMethodSrv, self._change_srv_cb
        )
        rospy.sleep(0.1)

    def _change_srv_cb(self, msg):
        rospy.loginfo("An exploration method change is requested")
        rospy.loginfo("Changing to %s method..." % msg.exploration_method)
        self.exploration_method = msg.exploration_method
        rospy.loginfo("Restarting counting process...")
        self.people_restart_srv()
        return ChangeMethodSrvResponse()

    def spin(self):
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            current = rospy.Time.now()
            # asking for exploration every self._exp_req_dur
            if (current - start) > self._exp_req_dur:
                self.request_exploration(current)
                start = current
            rospy.sleep(60)

    def request_exploration(self, start_time):
        end_time = start_time + self._exp_req_dur
        rospy.loginfo(
            "Requesting a visit between %d and %d"
            % (start_time.secs, end_time.secs)
        )
        budget = self.budget_control.get_allocated_budget(start_time, end_time)
        is_ubc = self.exploration_method == "ubc"
        people_estimates = self.people_srv(start_time, end_time, is_ubc, False)
        visit_plan = [
            (estimate, people_estimates.region_ids[ind]) for ind, estimate in enumerate(
                people_estimates.estimates)
        ]
        visit_plan = sorted(visit_plan, key=lambda i: i[0], reverse=True)
        if self.exploration_method != "ubc":
            visit_plan = self._check_visit_plan(
                start_time, end_time, visit_plan
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
            suggested_wps[:3], suggested_score[:3], budget
        )
        rospy.loginfo("Recommended waypoints to visit: %s" % str(task))
        return task

    def _check_visit_plan(self, start_time, end_time, visit_plan):
        scales = self.people_srv(start_time, end_time, False, True)
        scale_plan = list()
        for ind, scale in enumerate(scales.estimates):
            scale_plan.append((scale, scales.region_ids[ind]))
        if len(scale_plan) != 0:
            scale_plan = sorted(scale_plan, key=lambda i: i[0], reverse=True)
            lower_threshold = scale_plan[0][0] - (self.epsilon * scale_plan[0][0])
            high_visit = list()
            for total_scale, roi in scale_plan:
                if total_scale <= scale_plan[0][0] and total_scale >= lower_threshold:
                    high_visit.append(roi)
            p = len(high_visit) / float(len(scales.estimates))
            scale_plan = sorted(scale_plan, key=lambda i: i[0])
            if random.random() > p:
                rospy.loginfo("Changing WayPoints to visit unobserved places...")
                new_visit_plan = list()
                for i in scale_plan:
                    for j in visit_plan:
                        if i[1] == j[1]:
                            new_visit_plan.append(j)
                            break
                visit_plan = new_visit_plan
        return visit_plan

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

    def _get_waypoints_from_file(self):
        roi_wp_hashmap = yaml.load(
            open(
                roslib.packages.get_pkg_dir('activity_exploration') + '/config/region_to_wp.yaml',
                'r'
            )
        )
        return roi_wp_hashmap

if __name__ == '__main__':
    rospy.init_node("activity_exploration")
    ar = ActivityRecommender(rospy.get_name())
    ar.spin()
    rospy.spin()
