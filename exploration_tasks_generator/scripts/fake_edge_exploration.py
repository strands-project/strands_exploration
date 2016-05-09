#! /usr/bin/env python

import random
import rospy

from strands_exploration_msgs.srv import GetExplorationTasks, GetExplorationTasksResponse
from strands_navigation_msgs.srv import GetTopologicalMap


class FakeEdgeExploration(object):

    def __init__(self):
        self.get_top_map_srv=rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        rospy.Service("/exploration_services/edge_exp_srv", GetExplorationTasks, self.service_cb)
        
    def service_cb(self, request):
        res=GetExplorationTasksResponse(task_definition=[],
                                           task_score=[])
        topo_map=self.get_top_map_srv(rospy.get_param("/topological_map_name")).map
        for i in range(0,3):
            node=random.choice(topo_map.nodes)
            if node.name != "ChargingPoint":
                edge=random.choice(node.edges).edge_id
                res.task_definition.append(edge)
                res.task_score.append(random.random())
        return res
                    

    def main(self):
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('fake_edge_exploration')

    fee = FakeEdgeExploration()
    fee.main()
    
    
