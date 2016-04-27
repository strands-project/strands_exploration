#! /usr/bin/env python

import rospy

from strands_exploration_msgs.srv import GetExplorationTasks, GetExplorationTasksResponse


class FakeEdgeExploration(object):

    def __init__(self):
        rospy.Service("edge_exp_srv", GetExplorationTasks, self.service_cb)
        
    def service_cb(self, request):
        return GetExplorationTasksResponse(task_definition=['WayPoint19_WayPoint20', 'WayPoint21_WayPoint22'],
                                           task_score=[0.8,0.3])
                    

    def main(self):
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('fake_edge_exploration')

    fee = FakeEdgeExploration()
    fee.main()
    
    
