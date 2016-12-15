#! /usr/bin/env python

import rospy
from math import ceil

from strands_executive_msgs.msg import Task
from exploration_bid_manager.exploration_bidder import ExplorationBidder
from strands_exploration_msgs.msg import ExplorationSchedule
from strands_executive_msgs import task_utils
from tsc_routine.object_search_dfns import *
from tsc_routine.object_search_dfns_quasimodo import *

class SpatioTemporalBidder(object):

    def __init__(self):
        
        self.bidder = ExplorationBidder()
        self.grids_schedule = ExplorationSchedule()
        self.objects_schedule = ExplorationSchedule()
        self.bidder_timer = rospy.get_param("~bidder_timer",3600)
        self.rescheduleInterval = rospy.get_param('~rescheduleInterval', 86400)
        self.slot_duration = 0;
        self.num_slots = 0;
        
        #every 1 hour checks budget and bids
        rospy.Timer(rospy.Duration(self.bidder_timer), self.add_task)
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
                rospy.loginfo("Time slot duration: %d", self.slot_duration)

                #get current time slot:            
                nextSlot = self.getCurrentTimeSlot(self.slot_duration) + 1
                lookAhead = self.bidder_timer/self.slot_duration
            
                maxSlot = nextSlot + lookAhead

                if maxSlot - nextSlot > 1:        
                    #print "sorting schedules..."
                    self.schedule_sorted = []
                    #reorder slots based on the entropy for each 2h interval
                    for i in range(nextSlot, maxSlot):
                        e=dict()
                        
                        if self.grids_schedule.entropy[i] >= self.objects_schedule.entropy[i]:
                            e['nodeID']=self.grids_schedule.nodeID[i]
                            e['entropy']=self.grids_schedule.entropy[i]
                            e['mode']=self.grids_schedule.mode[i]
                        else:
                            e['nodeID']=self.objects_schedule.nodeID[i]
                            e['entropy']=self.objects_schedule.entropy[i]
                            e['mode']=self.objects_schedule.mode[i]
                        e['timeInfo']=self.grids_schedule.timeInfo[i]
                        self.schedule_sorted.append(e)
                    
                    self.schedule_sorted = sorted(self.schedule_sorted, key=lambda k:k['entropy'])
                
                    rospy.loginfo("Adding task...")
                    start_time = rospy.Time(secs = self.schedule_sorted[-1]['timeInfo'], nsecs = 0)
                    task_duration = rospy.Duration(secs = 60*15)                
                    task=Task(action= 'search_object',
                              start_node_id=self.schedule_sorted[-1]['nodeID'],
                            end_node_id=self.schedule_sorted[-1]['nodeID'],
                            start_after=start_time,
                            end_before=start_time + rospy.Duration(60*25),
                            max_duration=task_duration,
                            )
                     
                    if self.schedule_sorted[-1]['mode'] == 'object_full':
                        rois = get_object_search_dfn_quasimodo(self.schedule_sorted[-1]['nodeID'])
                    else:
                        rois = get_object_search_dfn(self.schedule_sorted[-1]['nodeID'])
                        
                    task_utils.add_string_argument(task, self.schedule_sorted[-1]['nodeID'])
                    task_utils.add_string_argument(task, rois[1])
                    task_utils.add_string_argument(task, rois[2])
                    task_utils.add_string_argument(task, self.schedule_sorted[-1]['mode'])
                                        
                    bidRatio = (self.num_slots - nextSlot)/(self.bidder_timer/self.slot_duration)
                    bid = int(ceil((self.bidder.available_tokens - self.bidder.currently_bid_tokens)/bidRatio))
                

                
                    if bid > 0:
                        rospy.loginfo("Bidder: Bid Amount: %d -- Node: %s -- Time: %d -- Mode: %s -- Start Time: %d -- Roi: %s -- Surface: %s",bid,
                                      self.schedule_sorted[-1]['nodeID'],
                                    self.schedule_sorted[-1]['timeInfo'], self.schedule_sorted[-1]['mode'],
                                    start_time.secs, rois[1],rois[2]) 
                        self.bidder.add_task_bid(task, bid)
                    else:
                        rospy.loginfo("Bidder: Bid value to low to add task!")
                        rospy.loginfo("Bidder: Current budget: %d", self.bidder.available_tokens)
                        
            else:
               rospy.loginfo("Bidder: Waiting for both schedules (3D changes and SOMA...)") 
        
     
    def main(self):
        
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('spatiotemporal_exploration_bidder')

    bidder = SpatioTemporalBidder()
    bidder.main()
    
    
