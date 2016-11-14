#! /usr/bin/env python

import rospy
from math import ceil

from strands_executive_msgs.msg import Task
from exploration_bid_manager.exploration_bidder import ExplorationBidder
from strands_exploration_msgs.msg import ExplorationSchedule
from strands_executive_msgs import task_utils

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
        self.add_task('foo', test='bar')
        
           
    def getCurrentTimeSlot(self, slot_duration):
        
        numSlots = 24*3600/slot_duration;
        now = rospy.get_rostime().secs
        
        currentSlot = (now-self.exploration_schedule.timeInfo[0])/slot_duration;
        
        if currentSlot > numSlots:
            currentSlot = numSlots
        
        return currentSlot
        
    def add_task(self, *args, **kwargs):
#        print "Availabe tokens: ", self.bidder.available_tokens
#        print "Currently bid tokens: ", self.bidder.currently_bid_tokens
                
        
        if len(self.exploration_schedule.timeInfo) == 0:
            print " exploration schedule no yet received"
        else:
            self.slot_duration = self.exploration_schedule.timeInfo[2] - self.exploration_schedule.timeInfo[1]
            print "Time slot duration: ", self.slot_duration

            #get current time slot:            
            nextSlot = self.getCurrentTimeSlot(self.slot_duration) + 1
            print "nextSlot: ", nextSlot
            lookAhead = self.bidder_timer/self.slot_duration
            print "lookAhead: ", lookAhead
            
            maxSlot = nextSlot + lookAhead
            print "maxSlot: ", maxSlot
            if maxSlot - nextSlot > 1:        
                print "sorting scheduling..."
                self.schedule_sorted = []
                #reorder slots based on the entropy for each 2h interval
                for i in range(nextSlot, maxSlot):
                    e=dict()
                    e['timeInfo']=self.exploration_schedule.timeInfo[i]
                    e['nodeID']=self.exploration_schedule.nodeID[i]
                    e['entropy']=self.exploration_schedule.entropy[i]
                    self.schedule_sorted.append(e)
                    
                self.schedule_sorted = sorted(self.schedule_sorted, key=lambda k:k['entropy'])
                
                print "adding schedule..."
                start_time = rospy.Time(secs = self.schedule_sorted[-1]['timeInfo'], nsecs = 0)
                end_time = rospy.Time(secs = self.schedule_sorted[-1]['timeInfo'] + self.slot_duration, nsecs = 0)
                task_duration = rospy.Duration(secs = self.bidder_timer)                
                task=Task(action= 'do_sweep',
                        start_node_id=self.schedule_sorted[-1]['nodeID'],
                        end_node_id=self.schedule_sorted[-1]['nodeID'],
                        start_after=start_time,
                        end_before=end_time,
                        max_duration=task_duration,
                        )
                        
                task_utils.add_string_argument(task, 'complete')
                                        
                bidRatio = (self.num_slots - nextSlot)/(self.bidder_timer/self.slot_duration)
                bid = int(ceil((self.bidder.available_tokens - self.bidder.currently_bid_tokens)/bidRatio))
                
                if bid > 0:
                    print "Bid Amount: ", bid, "Node: ", self.schedule_sorted[-1]['nodeID'], "Time: ", self.schedule_sorted[-1]['timeInfo'] 
                    self.bidder.add_task_bid(task, bid)
                else:
                    print "bid value to low to add task!"
                    print "current budget: ", self.bidder.available_tokens
        
     
    def main(self):
        
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('spatiotemporal_exploration_bidder')

    bidder = SpatioTemporalBidder()
    bidder.main()
    
    
