#! /usr/bin/env python

import rospy
from math import ceil

from strands_executive_msgs.msg import Task
from exploration_bid_manager.exploration_bidder import ExplorationBidder
from strands_exploration_msgs.msg import ExplorationSchedule

class SpatioTemporalBidder(object):

    def __init__(self):
        
        self.bidder = ExplorationBidder()
        self.exploration_schedule = ExplorationSchedule()
        self.slot_durantion = 0;
        self.num_slots = 0;
        
        rospy.Timer(rospy.Duration(60), self.add_task)
        rospy.Subscriber("/spatiotemporal_exploration/exploration_schedule", ExplorationSchedule, self.schedule_listener)
    
        ##DEBUGGING SERVICES
        #rospy.Service("add_budget", Empty, self.add_budget)
        #rospy.Service("bid", Empty, self.bid)
        #rospy.Service("get_info", Empty, self.get_info)
        
    def schedule_listener(self, data):
        self.exploration_schedule = data 
        
        self.slot_duration = self.exploration_schedule.timeInfo[2] - self.exploration_schedule.timeInfo[1]
        self.num_slots  = len(data.timeInfo)
        #some debug messages:        
        print "Nr slots: ", self.num_slots 
        print "Time slot duration: ", self.slot_durantion
           
#        self.schedule_sorted = []
#        for i in range(len(data.timeInfo)):
#            e={}
#            self.e['timeInfo']=data.timeInfo[i]
#            self.e['nodeID']=data.nodeID[i]
#            self.e['entropy']=data.entropy[i]
#            self.schedule_sorted.append(e)

    def add_task(self, timer_event):
        print self.bidder.available_tokens
        print self.bidder.currently_bid_tokens
        task=Task(action= 'do_sweep',
                        start_node_id='CorpLocker6',
                        end_node_id='CorpLocker6',
                        start_after=rospy.get_rostime(),
                        end_before=rospy.get_rostime() + rospy.Duration(60*1),
                        max_duration=rospy.Duration(1*60))
        bid = int(ceil(0.1*(self.bidder.available_tokens- self.bidder.currently_bid_tokens)))
        self.bidder.add_task_bid(task, bid)
        
    #DEBUGGING SERVICES    
    #def add_budget(self, req):
        #self.bidder.available_tokens+=20
        #self.bidder.process_task_queue()
        #return []
    
    #def bid(self, req):
        #task=Task(action= 'do_sweep',
                        #start_node_id='CorpLocker6',
                        #end_node_id='CorpLocker6',
                        #start_after=rospy.get_rostime(),
                        #end_before=rospy.get_rostime() + rospy.Duration(60*1),
                        #max_duration=rospy.Duration(2*60))
        #bid = 10
        #self.bidder.add_task_bid(task, bid)
        #return []
        
        
    #def get_info(self, req):
        #print "AVAILABLE TOKENS: ", self.bidder.available_tokens
        #print "BID TOKENS: ", self.bidder.currently_bid_tokens
        #print "ADDED TOKENS: ", self.bidder.currently_added_tokens
        #print "ADDED TASKS: ", self.bidder.added_tasks
        #print "--------------------------------------------------------------------------------------------------------------------"
        #print "QUEUED TASJS: ", self.bidder.queued_tasks
        #print "\n\n\n\n\n\n\n"
        #return []
     
    def main(self):
        
        #split the day in 4 tranches
        # for each tranche select 
        slots_tranches = self.num_slots/4 
        print "nr slots per tranch: ", slots_tranches

        for i in xrange(4):
            print "tranche: ", i
            for j in range(i*slots_tranches, i*slots_tranches+slots_tranches,1):
                tranch_budget = self.bidder.available_tokens
       # Wait for control-c
        rospy.spin()       



if __name__ == '__main__':
    rospy.init_node('spatiotemporal_exploration_bidder')

    bidder = SpatioTemporalBidder()
    bidder.main()
    
    
