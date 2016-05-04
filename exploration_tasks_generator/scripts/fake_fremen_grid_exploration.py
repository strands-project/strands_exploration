#! /usr/bin/env python

import random
import rospy

from strands_exploration_msgs.srv import GetExplorationTasks, GetExplorationTasksResponse
from strands_navigation_msgs.srv import GetTopologicalMap

class FakeFremenGridExploration(object):

    def __init__(self):
        self.get_top_map_srv=rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        rospy.Service("/exploration_services/fremen_grid_exp_srv", GetExplorationTasks, self.service_cb)
        
    def service_cb(self, request):
        res=GetExplorationTasksResponse(task_definition=[],
                                           task_score=[])
        topo_map=self.get_top_map_srv(rospy.get_param("/topological_map_name")).map
        for i in range(0,8):
            node=random.choice(topo_map.nodes)
            if node.name != "ChargingPoint":
                res.task_definition.append(node.name)
                res.task_score.append(random.random())
        return res
                    

    def main(self):
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('fake_fremen_grid_exploration')

    ffge = FakeFremenGridExploration()
    ffge.main()
    
    
