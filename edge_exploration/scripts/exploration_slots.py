#!/usr/bin/env python

import sys
import rospy
import pymongo
import numpy as np
import math
import copy

from strands_navigation_msgs.srv import *
from strands_exploration_msgs.srv import *
from strands_navigation_msgs.msg import TopologicalMap



class exploration_slots(object):

    def __init__(self):
        self.map_received =False
        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")
        self.mongo_client = pymongo.MongoClient(host, port)        
        
        
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        rospy.loginfo("Waiting for Topological map ...")       
        while not self.map_received:
            rospy.sleep(rospy.Duration(0.1))
        rospy.loginfo("... Got Topological map")        
        
        self.explore_srv=rospy.Service('/exploration_services/get_edge_exploration', strands_exploration_msgs.srv.GetExplorationTasks, self.get_exploration_task_cb)
        rospy.loginfo("All Done ...")
        rospy.spin()

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



    def get_exploration_task_cb(self, req):
        exptsk=[]
        tskscr=[]        
        
        print "exploration time"
        explotime = rospy.Duration.from_sec((req.end_time.secs - req.start_time.secs))
        timeslot = req.start_time + rospy.Duration.from_sec((req.end_time.secs - req.start_time.secs)/2)
        print timeslot.secs, explotime.secs
        
        
        
        ent = self.predict_entropy(timeslot)
        est = self.predict_edges(timeslot)
        self.fill_values(ent, est)
        self.estimate_scores()

        results = copy.copy(self.eids)
        results = sorted(results, key=lambda k: k['score'], reverse=True)

        task_time=0
        for i in results:
            if i['probs']>0.3:
                print task_time, i['time'].secs*2
                task_time+=(i['time'].secs*2)
                if task_time < explotime.secs:
                    print i['edge_id'], i['samples'], i['time'].secs, i['entropy'], i['score'], i['probs']
                    exptsk.append(i['edge_id'])
                    tskscr.append(i['score'])
                else:
                    break
        
        return exptsk, tskscr


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
            
#            print "+++++"
#            print i, self.eids[i]['edge_id'], self.eids[i]['samples'], self.eids[i]['entropy'], self.eids[i]['score']
            #print wsbs, self.eids[i]['entropy'], 1-wsbs, sbs, self.eids[i]['score']


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
        rospy.wait_for_service('/topological_prediction/edge_entropies')
        try:
            get_prediction = rospy.ServiceProxy('/topological_prediction/edge_entropies', strands_navigation_msgs.srv.PredictEdgeState)
            print "Requesting prediction for %f"%epoch.secs
            resp1 = get_prediction(epoch)
            print "got %d edge ids, %d probs"%(len(resp1.edge_ids),len(resp1.probs))
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def predict_edges(self, epoch):
        rospy.wait_for_service('/topological_prediction/predict_edges')
        try:
            get_prediction = rospy.ServiceProxy('/topological_prediction/predict_edges', strands_navigation_msgs.srv.PredictEdgeState)
            print "Requesting prediction for %f"%epoch.secs
            resp1 = get_prediction(epoch)
            print "got %d edge ids, %d probs"%(len(resp1.edge_ids),len(resp1.probs))
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        
if __name__ == "__main__":
    rospy.init_node('topological_prediction_test')       
    server = exploration_slots()