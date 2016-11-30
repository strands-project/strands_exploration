#! /usr/bin/env python

import rospy
import numpy as np
import pymongo
import math
import copy
import datetime

from strands_executive_msgs.msg import Task
from exploration_bid_manager.exploration_bidder import ExplorationBidder

from std_srvs.srv import Empty

from strands_navigation_msgs.srv import *

from strands_navigation_msgs.msg import TopologicalMap


class EdgeBider(object):

    def __init__(self):
        self.bidder = ExplorationBidder()
        self.map_received =False
        rospy.on_shutdown(self.shutdown)

        self.service_timeout = rospy.get_param("~timeout",10)
        self.time_of_slots = rospy.get_param("~slot_time",1200)
        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")
        self.mongo_client = pymongo.MongoClient(host, port)
        
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        rospy.loginfo("Waiting for Topological map ...")       
        while not self.map_received:
            rospy.sleep(rospy.Duration(0.1))
        rospy.loginfo("... Got Topological map")

        
        
        self.get_info()
        self.timer_obj = rospy.Timer(rospy.Duration(self.time_of_slots), self.get_exploration_task_cb)
        self.timer_obj2 = rospy.Timer(rospy.Duration(30), self.get_info_cb)
        self.get_exploration_task()
        rospy.spin()
        
    
    def get_exploration_task_cb(self, timer_events):
        self.get_exploration_task()
#        exptsk=[]
#        tskscr=[]        
        
    def get_exploration_task(self):
        exptsk=[]
        tskscr=[]
        tsktoad=[]
        print "exploration time"
        explotime = rospy.Duration.from_sec(self.time_of_slots)
        self.timeslot_start = rospy.Time.now()+rospy.Duration.from_sec(10)
        timeslot = self.timeslot_start+rospy.Duration.from_sec(self.time_of_slots/2)
        self.timeslot_end=self.timeslot_start+rospy.Duration.from_sec(self.time_of_slots)
        print timeslot.secs, explotime.secs
        
        ent = self.predict_entropy(timeslot)
        est = self.predict_edges(timeslot)
        self.fill_values(ent, est)
        self.estimate_scores()

        print self.eids
#        print self.edges_to_explote

        results = copy.copy(self.eids)
        results = sorted(results, key=lambda k: k['score'], reverse=True)

        task_time=0
        for i in results:
            if i['probs']>0.3:
                #print task_time, i['time'].secs*2
                task_time+=(i['time'].secs*4)
                if task_time < (explotime.secs/4):
                    #print i['edge_id'], i['samples'], i['time'].secs, i['entropy'], i['score'], i['probs']
                    exptsk.append(i['edge_id'])
                    tskscr.append(i['score'])
                else:
                    break
        
        #tokens_to_use = self.bidder.available_tokens/((3600*24)/self.time_of_slots)
        x_slots = np.linspace(0.0, 24.0, (3*24))
        myd=self.bidder.available_tokens*norm.pdf(x_slots, 12, 4)
        now = datetime.datetime.now()
        slot = now.hour*3+(int(np.floor(now.minute/20.0))+1)
        tokens_to_use= np.ceil(myd[slot])

        total_entropy = sum(tskscr)

        total_tokens=0
        for i in range(len(tskscr)):
            e={}
            tstr= exptsk[i].split('_')
            e['tokens']=int(math.ceil((tskscr[i]*tokens_to_use)/total_entropy))
            if e['tokens'] <= 0:
                e['tokens']=1
            total_tokens+=e['tokens']
            e['origin']=tstr[0]
            e['goal']=tstr[1]
            e['action']='(F ("'+tstr[0]+'" & (X "'+tstr[1]+'")))'
            tsktoad.append(e)

        #print tokens_to_use, total_tokens, total_entropy
        
        #print exptsk, tskscr
        #print tsktoad
        self.add_tasks(tsktoad)





    def estimate_scores(self):
        samples = np.array([x['samples'] for x in self.eids])
        mean = np.mean(samples)
        minimum = np.min(samples)
        maximum = np.max(samples)
        #print "Mean: %f Max: %f Min: %f"%(mean, minimum, maximum)
        print mean, minimum, maximum
        for i in range(len(self.eids)):
            if self.eids[i]['samples'] >= mean:
                sbs = (float(self.eids[i]['samples'])-maximum)/(2*(mean-maximum))
            else:
                sbs = 1 + (float(self.eids[i]['samples'])-minimum)/(2*(minimum-mean))
            
            if sbs >= 0.9 :
                wsbs = (math.sqrt(1-sbs))
                self.eids[i]['score'] = (wsbs*self.eids[i]['entropy'])+((1-wsbs)*sbs)
            else:
                self.eids[i]['score'] = self.eids[i]['entropy']#((1-(1/(float(self.eids[i]['samples'])+1)))*self.eids[i]['entropy'])+(1/(float(self.eids[i]['samples'])+1))

            if self.eids[i]['score'] < 0.0:
                self.eids[i]['score']=0.0
            elif self.eids[i]['score'] > 1.0:
                self.eids[i]['score']=1.0

            if self.eids[i]['samples'] <= 10 :
                self.eids[i]['probs'] = 0.5    
                
#            print "+++++"
#            print i, self.eids[i]['edge_id'], self.eids[i]['samples'], self.eids[i]['entropy'], self.eids[i]['score']
            #print wsbs, self.eids[i]['entropy'], 1-wsbs, sbs, self.eids[i]['score']        


#        #eids, ents, nothing = self.predict_entropy(timeslot)
#        v= self.predict_entropy(timeslot)
#        self.edges_to_explote=[]
#        for i in range(len(v.edge_ids)):
#            f={}
#            f['eid']=v.edge_ids[i]
#            f['ent']=v.probs[i]
#            self.edges_to_explote.append(f)
        
        

        
    def add_tasks(self, tasks_to_add):
        #print self.bidder.available_tokens
        #print self.bidder.currently_bid_tokens
        for i in tasks_to_add:  
            wp1=i['origin']
            wp2=i['goal']#+'"'
            task=Task(action= i['action'],start_node_id=wp1,end_node_id=wp2,start_after=self.timeslot_start,end_before=self.timeslot_end,max_duration=rospy.Duration(2*60))
            bid = i['tokens']
            self.bidder.add_task_bid(task, bid)



    def fill_values(self, entropies, estimations):
        db=self.mongo_client.message_store
        collection=db['nav_stats']        
        
        for i in range(len(entropies.edge_ids)):
            ind=self.edgid.index(entropies.edge_ids[i])
            self.eids[ind]['entropy']= entropies.probs[i]

        for i in range(len(estimations.edge_ids)):
            ind=self.edgid.index(estimations.edge_ids[i])
            self.eids[ind]['time']= estimations.durations[i]
            self.eids[ind]['probs']= estimations.probs[i]

        for i in range(len(self.eids)):
            query =  {'topological_map': self.top_map.pointset, 'edge_id':self.eids[i]['edge_id']}
            self.eids[i]['samples'] = collection.find(query).count()

        
    def predict_entropy(self, epoch):
        rospy.wait_for_service('/topological_prediction/edge_entropies', timeout=self.service_timeout)
        try:
            get_prediction = rospy.ServiceProxy('/topological_prediction/edge_entropies', strands_navigation_msgs.srv.PredictEdgeState)
            print "Requesting prediction for %f"%epoch.secs
            resp1 = get_prediction(epoch)
            print "got %d edge ids, %d probs"%(len(resp1.edge_ids),len(resp1.probs))
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e        


    def predict_edges(self, epoch):
        rospy.wait_for_service('/topological_prediction/predict_edges', timeout=self.service_timeout)
        try:
            get_prediction = rospy.ServiceProxy('/topological_prediction/predict_edges', strands_navigation_msgs.srv.PredictEdgeState)
            print "Requesting prediction for %f"%epoch.secs
            resp1 = get_prediction(epoch)
            print "got %d edge ids, %d probs"%(len(resp1.edge_ids),len(resp1.probs))
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def get_info_cb(self, events):
        self.get_info()

    def get_info(self):
        #print "AVAILABLE TOKENS: ", self.bidder.available_tokens
        #print "BID TOKENS: ", self.bidder.currently_bid_tokens
        #print "ADDED TOKENS: ", self.bidder.currently_added_tokens
        #print "ADDED TASKS: ", self.bidder.added_tasks
        #print "---------------------------------------------------------------"
        #print "QUEUED TASKS: ", self.bidder.queued_tasks
        #print "\n"
        return []

     
    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        self.top_map = msg
        self.map_received = True
        self.get_list_of_edges()


    def get_list_of_edges(self):
        self.edgid=[]
        self.eids=[]
        rospy.loginfo("Querying for list of edges")
        for i in self.top_map.nodes :
            for j in i.edges:
                if j.edge_id not in self.edgid :
                    self.edgid.append(j.edge_id)
                    val={}
                    val["edge_id"]=j.edge_id
                    val["entropy"]=0.0
                    val["time"]=0.0
                    val['samples']= 0
                    val['score']= 0.0
                    val['prob']= 0.0
                    self.eids.append(val)
        fdbmsg = 'Done. %d edges found' %len(self.edgid)
        rospy.loginfo(fdbmsg)


    def shutdown(self):
        self.timer_obj.shutdown()
        self.timer_obj2.shutdown()
        print "BYE"

if __name__ == '__main__':
    rospy.init_node('edge_bidder_node')
    bidder = EdgeBider()

    
    
