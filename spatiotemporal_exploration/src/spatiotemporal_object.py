#! /usr/bin/env python

import rospy

from strands_exploration_msgs.msg import ExplorationSchedule
#from frongo.srv import PredictState
from strands_navigation_msgs.srv import GetTaggedNodes
import frongo
import yaml


class SomaExploration():

    def __init__(self):
        
        self.db = rospy.get_param('db', 'message_store')
        self.collection = rospy.get_param('collection', 'view_stats')
        self.rescheduleInterval = rospy.get_param('rescheduleInterval', 86400)
        self.taskDuration = rospy.get_param('taskDuration', 1200)
        self.schedule_pub = rospy.Publisher('/object_exploration', ExplorationSchedule, queue_size=10)
        
        self.numSlots = 24*3600/self.taskDuration;
        
        #stores exploration schedule
        self.soma_schedule = ExplorationSchedule()
        
    
        #get tagged nodes
        try:
            rospy.wait_for_service('/topological_map_manager/get_tagged_nodes', timeout = 10)
            self.topo_nodes = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', GetTaggedNodes)
            data= self.topo_nodes('Exploration')
        except rospy.ServiceException, e:
                rospy.logerr("Service call failed %s. Is Frongo started?"%e)
    
        output_models=[]
        print data.nodes
        for i in data.nodes:
            d = self.base_model(i, db = self.db, collection = self.collection)
            output_models.append(d)
            
        yml = yaml.safe_dump(output_models, default_flow_style=False)
            
        try:
            rospy.wait_for_service('/frongo/add_model_defs', timeout=10)
            cont = rospy.ServiceProxy('/frongo/add_model_defs', frongo.srv.AddModel)
            resp1 = cont(yml)
            print resp1
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed %s. Is Frongo started?"%e)
        
        
        #exploration schedule:
        currentTime = rospy.Time.now()
        self.processSchedule(currentTime.secs)
        
        #every 5 minutes checks current time
        #after midnight creates new schedule
        rospy.Timer(rospy.Duration(5*60), self.checkSchedule)

        rospy.spin()        
         
    def checkSchedule(self, timer_event):
            
        #count current time slot!
        currentTime = rospy.Time.now()
        current_slot = self.getNextTimeSlot(currentTime.secs)
        print 'Time slot ', current_slot, ' of ', self.numSlots 
            
    def processSchedule(self, givenTime):
        
        schedule_msg = ExplorationSchedule()
        
        #look for schedule message on mongodb
        #...
        if self.loadSchedule(givenTime):
            print 'Loaded object exploration schedule successfully.'
        else:
            print 'Object exploration schedule not found. Creating a new one!'
            #if not find create one:
            self.generateNewSchedule(givenTime)
        
        #publish schedule file
        schedule_msg = self.soma_schedule
        print 'Publishing object exploration schedule'
        self.schedule_pub.publish(schedule_msg)  
        
        #save schedule (MongoDB)
        self.saveSchedule(givenTime)
                
    def generateNewSchedule(self, givenTime):
        
        #generate  new schedule
        new_schedule = ExplorationSchedule()        
        
        #1 -- update frongo models

        #2 -- entropies service

        #3 -- MC schedule 

        #store schedule!
        self.soma_schedule = new_schedule        

    
    def getNextTimeSlot(self, lookAhead):
        
        rescheduleCheckTime = 5;
        currentTime = rospy.Time.now()
        
        givenTime = currentTime.sec+lookAhead*self.taskDuration+rescheduleCheckTime;
        midnight = self.getMidnightTime(givenTime);
        
        if self.soma_schedule.timeInfo[0] == midnight:
            print 'Generating new schedule!'
            self.processSchedule(givenTime)
                
        currentSlot = (givenTime-self.soma_schedule.timeInfo[0])/self.taskDuration;
        
        if(currentSlot >= 0 and currentSlot < self.numSlots):
            return currentSlot
        else:
            print 'Object exploration schedule error: attempting to get task in a non-existent time slot.'
            return -1
            
    def loadSchedule(self, givenTime):
        print 'loading schedule'
        
    def saveSchedule(self, givenTime):
        print 'save schedule'
        
    def getMidnightTime(self, givenTime):
        
        midnight = (givenTime/self.rescheduleInterval) * self.rescheduleInterval
        return midnight
    
        
    def base_model(self, model_name, db="message_store", collection="message_store", timestamp_field="timestamp", 
               timestamp_type= "int32", data_field = "success", data_type = "boolean"):
                   
        d={}
        d['model']={}
        d['model']['name']= str('exp_'+model_name)
        d['model']['db']= str(db)
        d['model']['collection']= str(collection)
        d['model']['timestamp_field'] = str(timestamp_field)
        d['model']['timestamp_type'] = str(timestamp_type)
        d['model']['data_field'] = str(data_field)
        d['model']['data_type'] = str(data_type)
        d['model']['query'] = '{"waypoint":"'+model_name+'"}'
        
        return d
        

if __name__ == '__main__':
    

    rospy.init_node('soma_exploration')
    
    exploration = SomaExploration()
    
    
