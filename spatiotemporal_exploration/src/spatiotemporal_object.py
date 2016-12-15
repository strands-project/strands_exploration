#! /usr/bin/env python

import rospy

from strands_exploration_msgs.msg import ExplorationSchedule
from frongo.srv import PredictState
from strands_navigation_msgs.srv import GetTaggedNodes
import frongo
import yaml
import random
from std_srvs.srv import Trigger 
from mongodb_store.message_store import MessageStoreProxy

class SomaExploration():

    def __init__(self):
        
        self.db = rospy.get_param('~db', 'message_store')
        self.mode = rospy.get_param('~mode', 'object_mini')
        self.collection = rospy.get_param('~collection', 'view_stats')
        self.tag = rospy.get_param('~mini_exploration_tag', 'MiniExploration')
        self.rescheduleInterval = rospy.get_param('~rescheduleInterval', 86400)
        self.taskDuration = rospy.get_param('~taskDuration', 1200)
        self.schedule_pub = rospy.Publisher('/object_schedule', ExplorationSchedule, queue_size=10)
        self.max_entropy = 0.0
        self.numSlots = 24*3600/self.taskDuration;
        
        #stores exploration schedule
        self.soma_schedule = ExplorationSchedule()
        
    
        #get tagged nodes
        try:
            rospy.wait_for_service('/topological_map_manager/get_tagged_nodes', timeout = 10)
            self.topo_nodes = rospy.ServiceProxy('/topological_map_manager/get_tagged_nodes', GetTaggedNodes)
            self.data = self.topo_nodes(self.tag)
        except rospy.ServiceException, e:
                rospy.logerr("Service call failed %s. Is Frongo started?"%e)
    
        output_models=[]
        rospy.loginfo("Getting tagged nodes...")
        print self.data.nodes
        for i in self.data.nodes:
            d = self.base_model(i, db = self.db, collection = self.collection)
            output_models.append(d)
            
        yml = yaml.safe_dump(output_models, default_flow_style=False)
            
        rospy.loginfo("Creating frongo models...")
        try:
            rospy.wait_for_service('/frongo/add_model_defs', timeout=10)
            cont = rospy.ServiceProxy('/frongo/add_model_defs', frongo.srv.AddModel)
            resp1 = cont(yml)
            print resp1
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed %s. Is Frongo started?"%e)
        
        
        
        #exploration schedule:
        currentTime = rospy.Time.now()
        self.midnight = self.getMidnightTime(currentTime.secs)
        self.processSchedule(currentTime.secs)
        
        #every 5 minutes checks current time
        #after midnight creates new schedule
        rospy.Timer(rospy.Duration(10*60), self.checkSchedule)

        rospy.spin()        
         
    def checkSchedule(self, timer_event):
            
        #count current time slot!
        currentTime = rospy.Time.now()
        self.midnight = self.getMidnightTime(currentTime.secs)
        current_slot = self.getNextTimeSlot(1)
        print 'Time slot ', current_slot, ' of ', self.numSlots 
            
    def processSchedule(self, givenTime):
        
        #look for schedule message on mongodb
        #...
        if self.loadSchedule(self.midnight):
            rospy.loginfo("Loaded object exploration schedule successfully.")
            #publish schedule file
            rospy.loginfo("Publishing object exploration schedule")
            self.schedule_pub.publish(self.soma_schedule) 
        else:
            rospy.loginfo("Object exploration schedule not found. Creating a new one!")
            #if not find create one:
            self.generateNewSchedule(givenTime)
        

    def generateNewSchedule(self, givenTime):
        
        #generate  new schedule
        new_schedule = ExplorationSchedule()        
        
        #1 -- update frongo models
        print 'updating models'
        try:
            rospy.wait_for_service('/frongo/rebuild_all_models', timeout = 10)
            trigger = rospy.ServiceProxy('/frongo/rebuild_all_models', Trigger)
            trigger()
        except rospy.ServiceException, e:
                rospy.logerr("Service call failed %s. Is Frongo started?"%e)
        
        #2 -- entropies service

        #get epochs for each time slot
        times = []
        for i in range(self.numSlots):      
            t = int(self.midnight+3600*24/self.numSlots*i)
            #print' slot: ', i, ' epoch: ',t
            times.append(t)
            
        #query entropies for each waypoint and 
        time_slots = []
        
        for i in range(0,len(self.data.nodes)):
            entropies = {}
            entropies['waypoint'] = self.data.nodes[i]
            #print self.data.nodes[i]
            print 'Predicitng states of waypoint: ', self.data.nodes[i]
            
            try:
                rospy.wait_for_service('/frongo/get_entropies', timeout = 10)
                predictions = rospy.ServiceProxy('/frongo/get_entropies', PredictState)
                sassa = str('exp_'+ self.data.nodes[i] )
                resp = predictions(sassa, times)
                #print resp
                entropies['entropies'] = resp.predictions
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed %s. Is Frongo started?"%e)
            
            time_slots.append(entropies)            
            
        
        #3 -- MC schedule
        rospy.loginfo("Creating schedule")
        
        for s in range(self.numSlots): 
            wheel= []
            lastWheel = 0
            for i in range(len(self.data.nodes)):
                lastWheel += time_slots[i]['entropies'][s]
                wheel.append(lastWheel)

            randomNum = 0.0
            randomNum = random.random()*lastWheel            
            p = 0
            while(p < len(self.data.nodes) and randomNum > wheel[p]):
                p += 1
                
            if time_slots[p]['entropies'][s] >= self.max_entropy:
                    self.max_entropy = time_slots[p]['entropies'][s]
            

            new_schedule.entropy.append(time_slots[p]['entropies'][s])
            new_schedule.nodeID.append(time_slots[p]['waypoint'])
            new_schedule.mode.append(self.mode)
            new_schedule.timeInfo.append(times[s])
            
        new_schedule.midnight = times[0]

        #4 -- store schedule!
        self.soma_schedule = new_schedule
        
        print 'printing schedule: '
        for s in range(self.numSlots):
            print 'Time: ', self.soma_schedule.timeInfo[s], ' WayPoint: ', self.soma_schedule.nodeID[s], ' Entropy: ', self.soma_schedule.entropy[s]
        
        self.saveSchedule()
        
        #publish schedule file
        schedule_msg = self.soma_schedule
        rospy.loginfo("Publishing object exploration schedule.")
        self.schedule_pub.publish(schedule_msg) 

    def getNextTimeSlot(self, lookAhead):
        
        rescheduleCheckTime = 5;
        currentTime = rospy.Time.now()
        
        givenTime = currentTime.secs +lookAhead*self.taskDuration+rescheduleCheckTime;
        
        if not self.soma_schedule.timeInfo[0] == self.midnight:
            rospy.loginfo("Generating new schedule!")
            self.processSchedule(givenTime)
                
        currentSlot = (givenTime-self.soma_schedule.timeInfo[0])/self.taskDuration;
        
        if(currentSlot >= 0 and currentSlot < self.numSlots):
            return currentSlot
        else:
            rospy.loginfo("Object exploration schedule error: attempting to get task in a non-existent time slot.")
            return -1
            
    def loadSchedule(self, givenTime):
        rospy.loginfo("Looking for schedules in MongoDB.")
        
        msg_store = MessageStoreProxy(collection='exploration_schedules')    
        query = {"midnight" : givenTime}
        
        available = msg_store.query(ExplorationSchedule._type, query, {})
        
        
        if len(available) < 1:
             succeded = False
             rospy.loginfo("No schedules were found!")
        else:
            # Iterate through available array.
            # Construct message using K:V pairs.
            succeded = True
            rospy.loginfo("Schedule found... loading and publishing message!")
            load_schedule = ExplorationSchedule()
            #print available[0][0].timeInfo
            load_schedule.timeInfo = available[0][0].timeInfo
            load_schedule.entropy = available[0][0].entropy
            load_schedule.nodeID = available[0][0].nodeID
            load_schedule.midnight = available[0][0].midnight
            load_schedule.mode = available[0][0].mode
            self.soma_schedule = load_schedule

    
        return succeded
        
    def saveSchedule(self):
        rospy.loginfo("Saving schedule to MongoDb")

        msg_store = MessageStoreProxy(collection='exploration_schedules')        
        msg_store.insert(self.soma_schedule,{})
        
        
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
    
    
