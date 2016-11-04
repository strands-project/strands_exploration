#! /usr/bin/env python

import rospy
from math import ceil

from strands_executive_msgs.msg import Task
from exploration_bid_manager.exploration_bidder import ExplorationBidder
from strands_exploration_msgs.msg import ExplorationSchedule
from mongodb_store.message_store import  StringPair

class SpatioTemporalBidder(object):

    def __init__(self):
        
        self.bidder = ExplorationBidder()
        self.exploration_schedule = ExplorationSchedule()
        self.bidder_timer = rospy.get_param("~bidder_timer",3600)
        self.slot_duration = 0;
        self.num_slots = 0;
        
        #every 2 hours checks budget and bids
        rospy.Timer(rospy.Duration(self.bidder_timer), self.add_task)#7200
        rospy.Subscriber("/exploration_schedule", ExplorationSchedule, self.schedule_listener)
    

    def schedule_listener(self, data):
        self.exploration_schedule = data
        self.num_slots  = len(data.timeInfo)
        #some debug messages:        
        print "Received schedule -- nr of time slots: ", self.num_slots 
        
           
    def getCurrentTimeSlot(self, slot_duration):
        
        numSlots = 24*3600/slot_duration;
        now = rospy.get_rostime().secs
        
        currentSlot = (now-self.exploration_schedule.timeInfo[0])/slot_duration;
        
        if currentSlot > numSlots:
            currentSlot = numSlots
        
        return currentSlot
        
    def add_task(self, timer_event):
        print self.bidder.available_tokens
        print self.bidder.currently_bid_tokens
                
                
        if len(self.exploration_schedule.timeInfo) == 0:
            print " exploration schedule no yet received"
        else:
            self.slot_duration = self.exploration_schedule.timeInfo[2] - self.exploration_schedule.timeInfo[1]
            print "Time slot duration: ", self.slot_duration

            #get current time slot:            
            nextSlot = self.getCurrentTimeSlot(self.slot_duration) + 1
            lookAhead = self.bidder_timer/self.slot_duration
            
            maxSlot = nextSlot + lookAhead
            if maxSlot - nextSlot > 1:             
                #reorder slots based on the entropy for each 2h interval
                for i in range(nextSlot, maxSlot):
                    self.schedule_sorted = []
                    e={}
                    self.e['timeInfo']=self.exploration_schedule.timeInfo[i]
                    self.e['nodeID']=self.exploration_schedule.nodeID[i]
                    self.e['entropy']=self.exploration_schedule.entropy[i]
                self.schedule_sorted.append(e)
                                
                taskArg = StringPair()
                taskArg.second = "complete"
                
                task=Task(action= 'do_sweep',
                        start_node_id=self.schedule_sorted.nodeID[-1],
                        end_node_id=self.schedule_sorted.nodeID[-1],
                        start_after=self.schedule_sorted.timeInfo[-1],
                        end_before=self.schedule_sorted.timeInfo[-1] + self.slot_duration,
                        max_duration=self.slot_duration,
                        arguments = taskArg)
                                        
                bidRatio = (self.num_slots - nextSlot)/self.bidder_timer        
                bid = int(ceil((self.bidder.available_tokens - self.bidder.currently_bid_tokens)*bidRatio))
                self.bidder.add_task_bid(task, bid)
        
     
    def main(self):
        
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('spatiotemporal_exploration_bidder')

    bidder = SpatioTemporalBidder()
    bidder.main()
    
    
