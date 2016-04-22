#!/usr/bin/env python

import sys
import rospy

from mongodb_store.message_store import MessageStoreProxy
from std_msgs.msg import String
from topological_exploration.srv import *


class grid_store(object):

    def __init__(self) :

        self.save_grid_srv=rospy.Service('/save_grid', topological_exploration.srv.SaveGrid, self.save_grid_cb)
        self.load_grid_srv=rospy.Service('/save_grid', topological_exploration.srv.SaveGrid, self.load_grid_cb)
        rospy.loginfo("All Done ...")
        rospy.spin()


    def save_grid_cb(self, req):
        meta ={}
        msg_store = MessageStoreProxy(collection='strings') #change this
        msg_store.insert(req,meta)

    def load_grid_cb(self, req):
        msg_store = MessageStoreProxy(collection='door_stats')

        query_meta={}

        message_list = msg_store.query(std_msgs.msg.String, {}, query_meta)    



if __name__ == '__main__':
    rospy.init_node('grid_store')
    server = grid_store()