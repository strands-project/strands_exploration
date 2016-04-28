#! /usr/bin/env python

import rospy

from strands_exploration_msgs.srv import GetExplorationTasks, GetExplorationTasksResponse


class FakeFremenGridExploration(object):

    def __init__(self):
        rospy.Service("fremen_grid_exp_srv", GetExplorationTasks, self.service_cb)
        
    def service_cb(self, request):
        return GetExplorationTasksResponse(task_definition=['WayPoint17', 'Station'],
                                           task_score=[0.8,0.3])
                    

    def main(self):
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('fake_fremen_grid_exploration')

    ffge = FakeFremenGridExploration()
    ffge.main()
    
    
