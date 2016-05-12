#! /usr/bin/env python

import rospy
from numpy.random import choice, shuffle

from exploration_tasks_generator.exploration_services_manager import ExplorationServicesManager
from strands_executive_msgs.srv import AddTasks

class TaskGenerator(object):

    def __init__(self):
        self.trigger_dur=rospy.Duration(60*60)
        self.window_dur=rospy.Duration(60*20)
        self.slack_dur=rospy.Duration(60*2)
        
        self.tasks_per_trigger_dict={'edge_exploration':10, 'fremen_grid_exploration':8, 'activity_exploration':1} #TODO: make param
        self.default_tasks_per_trigger=2
        
        self.task_priorities={'activity_exploration':10}
        self.default_task_priority=0
        
        self.services_manager=ExplorationServicesManager()
        self.timer=rospy.Timer(self.trigger_dur, self.add_tasks_to_schedule)
        self.add_tasks_srv=rospy.ServiceProxy('/robot_routine/add_tasks', AddTasks)
        rospy.sleep(5)
        self.add_tasks_to_schedule(None)
        
    
    def add_tasks_to_schedule(self, timer_event):
        i=rospy.Duration(0)
        task_list=[]
        current_time=rospy.get_rostime()
        while i < self.trigger_dur:
            task_list+=self.services_manager.generate_tasks(current_time+i+self.slack_dur,
                                                            current_time+i+self.slack_dur+self.window_dur)
            i+=self.window_dur
        tasks_to_add=self.choose_tasks(task_list)
        print "ADDING", tasks_to_add
        self.add_tasks_srv(tasks_to_add)
        
    def choose_tasks(self, task_list):
        tasks_to_add=[]
        task_types=self.get_task_types(task_list)
        for task_type in task_types:
            current_task_list=self.task_list_by_task_type(task_list, task_type)
            if self.tasks_per_trigger_dict.has_key(task_type):
                n_tasks=min(len(current_task_list), self.tasks_per_trigger_dict[task_type])
            else:
                n_tasks=min(len(current_task_list), self.default_tasks_per_trigger)
            for i in range(0, n_tasks):
                chosen_task=self.pick_task(current_task_list)
                if self.task_priorities.has_key(task_type):
                    chosen_task.task_def.priority=self.task_priorities[task_type]
                else:
                    chosen_task.task_def.priority=self.default_task_priority
                tasks_to_add.append(chosen_task)
        shuffle(tasks_to_add)
        return [task.task_def for task in tasks_to_add]
    
    def pick_task(self, task_list):
        #normalise weights
        total=0
        for task in task_list:
            total+=task.score
        if total>0:
            weights=[task.score/total for task in task_list]
        else:
            weights=[1.0/len(task_list) for task in task_list]
        
        #weighted choice
        res=choice(task_list, p=weights)
        
        #remove
        index = task_list.index(res)
        del task_list[index]
        
        print res.task_type, res.task_def.start_after
        return res
                
    
    def get_task_types(self, task_list):
        res=set()
        for task in task_list:
            res.add(task.task_type)
        return res
    
    def task_list_by_task_type(self, task_list, task_type):
        res=[]
        for task in task_list:
            if task.task_type==task_type:
                res.append(task)
        return res

    def main(self):
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('exploration_tasks_generator_node')

    generator = TaskGenerator()
    generator.main()
    
    
