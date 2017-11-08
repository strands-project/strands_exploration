#!/usr/bin/env python

import rospy
import datetime

from popp_exploration.bidding_manager import BiddingManager

from strands_executive_msgs.msg import Task
from strands_executive_msgs import task_utils
from strands_navigation_msgs.msg import TopologicalMap

from region_observation.util import is_intersected
from region_observation.util import robot_view_cone, get_soma_info


class POPPExploration(object):

    def __init__(self, minimum_bid=500):
        rospy.loginfo("Initiating POPP exploration...")
        self.visited_places = list()
        self.current_date = datetime.datetime.fromtimestamp(
            rospy.Time.now().secs
        ).date()
        self.minimum_bid = minimum_bid
        self.soma_config = rospy.get_param(
            "~soma_config", "poisson_activity"
        )
        self.exploration_request_interval = rospy.Duration(
            rospy.get_param("~exploration_request_interval", 600)
        )
        self.exploration_duration = rospy.Duration(
            rospy.get_param("~exploration_duration", "600")
        )
        exploration_interval = rospy.Duration(self.exploration_duration.secs*3)
        self.bidding_manager = BiddingManager(
            exploration_interval=exploration_interval,
            minimum_bid=self.minimum_bid
        )
        # regions
        self.topo_map = None
        self.region_wps = self.region_to_waypoint_mapping(self.soma_config)
        rospy.loginfo(
            "Region ids and their nearest waypoints: %s" % str(self.region_wps)
        )
        # self.request_exploration(None)
        rospy.Timer(self.exploration_request_interval, self.request_exploration)

    def request_exploration(self, event):
        current_date = datetime.datetime.fromtimestamp(rospy.Time.now().secs).date()
        if current_date > self.current_date:
            self.current_date = current_date
            self.visited_places = list()
        self.bidding_manager.calculate_bidding_plan(self.region_wps.keys(), self.visited_places)
        for (start, roi, bid) in self.bidding_manager.bidding_plan:
            wp = self.region_wps[roi]
            start_time = start - self.exploration_duration
            duration = self.exploration_duration
            if bid >= self.minimum_bid:
                end_time = start + self.exploration_duration + self.exploration_duration
                task = Task(
                    action="wait_action", start_node_id=wp, end_node_id=wp,
                    start_after=start_time, end_before=end_time, max_duration=duration
                )
                task_utils.add_time_argument(task, rospy.Time())
                task_utils.add_duration_argument(task, duration)
                readable_time = datetime.datetime.fromtimestamp(start_time.secs).time()
                rospy.loginfo(
                    "Task to be requested: {wp:%s, roi:%s, start:%s, duration:%d, bid:%d}" % (
                        wp, roi, readable_time, duration.secs, int(bid)
                    )
                )
                self.bidding_manager.bidder.add_task_bid(task, int(bid))
                self.visited_places.append((start, roi))
            else:
                readable_time = datetime.datetime.fromtimestamp(start_time.secs).time()
                rospy.loginfo(
                    "Task: {wp:%s, roi:%s, start:%s, duration:%d, bid:%d} is dropped due to insufficient bidding bid" % (
                        wp, roi, readable_time, duration.secs, int(bid)
                    )
                )
        rospy.loginfo("Finish adding tasks...")

    def _topo_map_cb(self, topo_map):
        self.topo_map = topo_map

    def region_to_waypoint_mapping(self, soma_config):
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
    rospy.init_node("popp_exploration")
    ar = POPPExploration()
    rospy.spin()
