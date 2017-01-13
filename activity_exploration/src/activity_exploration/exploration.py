#!/usr/bin/env python


import yaml
import rospy
import roslib
import datetime
from std_srvs.srv import Empty

from activity_exploration.srv import ChangeMethodSrv
from activity_exploration.srv import ChangeMethodSrvResponse
from activity_exploration.budget_control import BudgetControl

from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
from strands_navigation_msgs.msg import TopologicalMap

from region_observation.util import is_intersected
from region_observation.util import robot_view_cone, get_soma_info


class ActivityRecommender(object):

    def __init__(self, minimal_bidding=500):
        rospy.loginfo("Initiating activity exploration...")
        self.visited_places = list()
        self.current_date = datetime.datetime.fromtimestamp(
            rospy.Time.now().secs
        ).date()
        self.minimal_bidding = minimal_bidding
        self.soma_config = rospy.get_param(
            "~soma_config", "activity_exploration"
        )
        self.exploration_method = rospy.get_param("~exploration_method", "ubc")
        self._exp_req_dur = rospy.Duration(
            rospy.get_param("~exploration_update_interval", 600)
        )
        self.exploration_duration = rospy.Duration(
            rospy.get_param("~exploration_duration", "600")
        )
        observe_interval = rospy.Duration(self.exploration_duration.secs*3)
        self.budget_control = BudgetControl(observe_interval=observe_interval)
        # all services to counters
        people_srv_name = rospy.get_param(
            "~people_srv", "/people_counter/people_best_time_estimate"
        )
        if people_srv_name != "":
            people_srv_name = "/" + people_srv_name.split("/")[1] + "/restart"
            rospy.loginfo("Connecting to %s service..." % people_srv_name)
            self._people_srv = rospy.ServiceProxy(people_srv_name, Empty)
            self._people_srv.wait_for_service()
        act_srv_name = rospy.get_param(
            "~activity_srv", "/activity_counter/activity_best_time_estimate"
        )
        if act_srv_name != "":
            act_srv_name = "/" + act_srv_name.split("/")[1] + "/restart"
            rospy.loginfo("Connecting to %s service..." % act_srv_name)
            self._act_srv = rospy.ServiceProxy(act_srv_name, Empty)
            self._act_srv.wait_for_service()
        scene_srv_name = rospy.get_param(
            "~scene_srv", "/scene_counter/scene_best_time_estimate"
        )
        if scene_srv_name != "":
            scene_srv_name = "/" + scene_srv_name.split("/")[1] + "/restart"
            rospy.loginfo("Connecting to %s service..." % scene_srv_name)
            self._scene_srv = rospy.ServiceProxy(scene_srv_name, Empty)
            self._scene_srv.wait_for_service()
        # regions
        self.epsilon = 0.15
        self.topo_map = None
        if rospy.get_param("~with_config_file", False):
            self.region_wps = self._get_waypoints_from_file()
        else:
            self.region_wps = self._get_waypoints(self.soma_config)
        rospy.loginfo(
            "Region ids and their nearest waypoints: %s" % str(self.region_wps)
        )
        rospy.sleep(0.1)
        rospy.Service(
            '%s/change_method_srv' % rospy.get_name(),
            ChangeMethodSrv, self._change_srv_cb
        )
        # self.request_exploration(None)
        rospy.Timer(self._exp_req_dur, self.request_exploration)

    def _change_srv_cb(self, msg):
        rospy.loginfo("An exploration method change is requested")
        rospy.loginfo("Changing to %s method..." % msg.exploration_method)
        self.exploration_method = msg.exploration_method
        rospy.loginfo("Restarting all counting processes...")
        self._act_srv()
        self._scene_srv()
        self._people_srv()
        return ChangeMethodSrvResponse()

    def request_exploration(self, event):
        current_date = datetime.datetime.fromtimestamp(rospy.Time.now().secs).date()
        if current_date > self.current_date:
            self.current_date = current_date
            self.visited_places = list()
        self.budget_control.get_budget_alloc(self.region_wps.keys(), self.visited_places)
        for (start, roi, budget) in self.budget_control.budget_alloc:
            wp = self.region_wps[roi]
            start_time = start - self.exploration_duration - self.exploration_duration
            duration = self.exploration_duration
            if budget >= self.minimal_bidding:
                end_time = start + self.exploration_duration + self.exploration_duration + self.exploration_duration
                task = Task(
                    action="record_skeletons", start_node_id=wp, end_node_id=wp,
                    start_after=start_time, end_before=end_time, max_duration=duration
                )
                task_utils.add_duration_argument(task, duration)
                task_utils.add_string_argument(task, roi)
                task_utils.add_string_argument(task, self.soma_config)
                readable_time = datetime.datetime.fromtimestamp(start_time.secs).time()
                rospy.loginfo(
                    "Task to be requested: {wp:%s, roi:%s, start:%s, duration:%d, budget:%d}" % (
                        wp, roi, readable_time, duration.secs, int(budget)
                    )
                )
                self.budget_control.bidder.add_task_bid(task, int(budget))
                self.visited_places.append((start, roi))
            else:
                readable_time = datetime.datetime.fromtimestamp(start_time.secs).time()
                rospy.loginfo(
                    "Task: {wp:%s, roi:%s, start:%s, duration:%d, budget:%d} is dropped due to insufficient bidding budget" % (
                        wp, roi, readable_time, duration.secs, int(budget)
                    )
                )
        rospy.loginfo("Finish adding tasks...")

    # def _check_visit_plan(self, start_time, end_time, visit_plan):
    #     scales = self.people_srv(start_time, end_time, False, True)
    #     scale_plan = list()
    #     for ind, scale in enumerate(scales.estimates):
    #         scale_plan.append((scale, scales.region_ids[ind]))
    #     if len(scale_plan) != 0:
    #         scale_plan = sorted(scale_plan, key=lambda i: i[0], reverse=True)
    #         lower_threshold = scale_plan[0][0] - (self.epsilon * scale_plan[0][0])
    #         high_visit = list()
    #         for total_scale, roi in scale_plan:
    #             if total_scale <= scale_plan[0][0] and total_scale >= lower_threshold:
    #                 high_visit.append(roi)
    #         p = len(high_visit) / float(len(scales.estimates))
    #         scale_plan = sorted(scale_plan, key=lambda i: i[0])
    #         if random.random() > p:
    #             rospy.loginfo("Changing WayPoints to visit unobserved places...")
    #             new_visit_plan = list()
    #             for i in scale_plan:
    #                 for j in visit_plan:
    #                     if i[1] == j[1]:
    #                         new_visit_plan.append(j)
    #                         break
    #             visit_plan = new_visit_plan
    #     return visit_plan

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
    ar = ActivityRecommender()
    rospy.spin()
