#! /usr/bin/env python

import rospy
from math import ceil

from strands_executive_msgs.msg import Task
from exploration_bid_manager.exploration_bidder import ExplorationBidder
from strands_exploration_msgs.msg import ExplorationSchedule
from strands_executive_msgs import task_utils
from tsc_routine.object_search_dfns import *
from tsc_routine.object_search_dfns_quasimodo import *
import random

class SpatioTemporalBidder(object):

    def __init__(self):
        
        self.bidder = ExplorationBidder()
        self.grids_schedule = ExplorationSchedule()
        self.objects_schedule = ExplorationSchedule()
        self.bidder_timer = rospy.get_param("~bidder_timer",3600)
        self.rescheduleInterval = rospy.get_param('~rescheduleInterval', 86400)
        self.slot_duration = 0;
        self.num_slots = 0;
        self.last_bid = 0;
        
        #every 5 min check current time slot
        rospy.Timer(rospy.Duration(60*5), self.add_task)
        rospy.Subscriber("/exploration_schedule", ExplorationSchedule, self.schedule_grids_listener)
        rospy.Subscriber("/object_schedule", ExplorationSchedule, self.schedule_objects_listener)
        
    
    def getMidnightTime(self, givenTime):        
        midnight = (givenTime/self.rescheduleInterval) * self.rescheduleInterval
        return midnight
    

    def schedule_grids_listener(self, data):
        self.grids_schedule = data
        self.num_slots  = len(data.timeInfo)
        #some debug messages:        
        rospy.loginfo("Bidder: Received 3D grids schedule -- nr of time slots: %d", self.num_slots)
        self.add_task('foo', test='bar')
        
    def schedule_objects_listener(self, data):
        self.objects_schedule = data
        self.num_slots  = len(data.timeInfo)
        #some debug messages:        
        rospy.loginfo("Bidder: Received object schedule -- nr of time slots: %d", self.num_slots)
        self.add_task('foo', test='bar')
        
           
    def getCurrentTimeSlot(self, slot_duration):
        
        numSlots = 24*3600/slot_duration;
        now = rospy.get_rostime().secs
        
        currentSlot = (now-self.grids_schedule.timeInfo[0])/slot_duration;
        
        if currentSlot > numSlots:
            currentSlot = numSlots
        
        return currentSlot
        
    def add_task(self, *args, **kwargs):
#        print "Availabe tokens: ", self.bidder.available_tokens
#        print "Currently bid tokens: ", self.bidder.currently_bid_tokens
                
        currentTime = rospy.Time.now()
        midnight = self.getMidnightTime(currentTime.secs)
        #print 'Current time: ', currentTime.secs, ' Midnight: ', midnight
        
        if len(self.grids_schedule.timeInfo) == 0 or len(self.objects_schedule.timeInfo) == 0:
            rospy.loginfo("Bidder: waiting for both schedules.")
        else:
            
            if self.grids_schedule.timeInfo[0] == midnight and self.objects_schedule.timeInfo[0] == midnight:
                rospy.loginfo("Received schedules for both explorations.")
                
                self.slot_duration = self.grids_schedule.timeInfo[2] - self.grids_schedule.timeInfo[1]
                #rospy.loginfo("Time slot duration: %d", self.slot_duration)

                #get current time slot:            
                currentSlot = self.getCurrentTimeSlot(self.slot_duration)
                
                rospy.loginfo("Time slot: %d/%d", currentSlot, self.num_slots)
                task_duration = rospy.Duration(secs = 60*10)
                
                if currentSlot is not self.last_bid:
                    if self.num_slots - currentSlot > 2:
                        rospy.loginfo("Adding task...")
                        
                        #choose between full and mini:
                        if random.randint(0,2) == 0:
                            #start_time = rospy.Time(secs = self.grids_schedule[currentSlot+1]['timeInfo'], nsecs = 0)
                            start_time = rospy.Time(secs = self.grids_schedule.timeInfo[currentSlot+1], nsecs = 0)
                            node_observation = self.grids_schedule.nodeID[currentSlot+1]
                            mode = self.grids_schedule.mode[currentSlot+1]
                        else:
                            start_time = rospy.Time(secs = self.objects_schedule.timeInfo[currentSlot+1], nsecs = 0)
                            node_observation = self.objects_schedule.nodeID[currentSlot+1]
                            mode = self.objects_schedule.mode[currentSlot+1]
                        
                        #add task:          
                        task=Task(action= 'search_object',
                                start_node_id=node_observation,
                                end_node_id=node_observation,
                                start_after=start_time,
                                end_before=start_time + rospy.Duration(60*15),
                                max_duration=task_duration,
                                )
                         
                        if mode == 'object_full':
                            rois = get_object_search_dfn_quasimodo(node_observation)
                        else:
                            rois = get_object_search_dfn(node_observation)
                            
                        task_utils.add_string_argument(task, node_observation)
                        task_utils.add_string_argument(task, rois[1])
                        task_utils.add_string_argument(task, rois[1])
                        task_utils.add_string_argument(task, mode)
                                            
                        
                        bidRatio = self.num_slots - currentSlot
                        bid = int(ceil((self.bidder.available_tokens - self.bidder.currently_bid_tokens)/bidRatio))
    
                        if bid > 0:
                            rospy.loginfo("Bidder: Bid Amount: %d -- Node: %s -- Start Time: %d -- Mode: %s -- Roi: %s -- Surface: %s",bid,
                                          node_observation,
                                        start_time.secs, mode, rois[1], rois[2]) 

                            self.bidder.add_task_bid(task, bid)
                            self.last_bid = currentSlot
                        else:
                            rospy.loginfo("Bidder: Bid value to low to add task!")
                            rospy.loginfo("Bidder: Current budget: %d", self.bidder.available_tokens)
                    else:
                        rospy.loginfo("Last slot of the day, waiting for midnight!")
        
 
            else:
               rospy.loginfo("Bidder: Waiting for both schedules (3D changes and SOMA...)") 
        
     
    def main(self):
        
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('spatiotemporal_exploration_bidder')

    bidder = SpatioTemporalBidder()
    bidder.main()
    
    
