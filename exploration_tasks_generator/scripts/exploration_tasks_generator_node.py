#! /usr/bin/env python

import rospy
import random

from exploration_tasks_generator.exploration_services_manager import ExplorationServicesManager
from strands_executive_msgs.srv import AddTask

class TaskGenerator(object):

    def __init__(self):
        self.services_manager=ExplorationServicesManager()
        self.timer=rospy.Timer(rospy.Duration(60*20), self.add_tasks_to_schedule)
        self.add_task_srv=rospy.ServiceProxy('/task_executor/add_task', AddTask)
        
    
    def add_tasks_to_schedule(self, timer_event):
        task_list=generator.services_manager.generate_tasks(rospy.get_rostime()+rospy.Duration(2),rospy.get_rostime()+rospy.Duration(30*60))
        random.shuffle(task_list)
        for task in task_list:
            self.add_task_srv(task.task_def)
            print "ADDED"
        
        

    def main(self):
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('exploration_tasks_generator_node')

    generator = TaskGenerator()
    generator.main()
    
    
