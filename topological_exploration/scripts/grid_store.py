#!/usr/bin/env python

import rospy

from mongodb_store.message_store import MessageStoreProxy
import strands_exploration_msgs.srv
import strands_exploration_msgs.msg

class grid_store(object):

    def __init__(self) :

        self.save_grid_srv=rospy.Service('/topological_exploration/save_grid', strands_exploration_msgs.srv.SaveGrid, self.save_grid_cb)
        self.load_grid_srv=rospy.Service('/topological_exploration/load_grid', strands_exploration_msgs.srv.LoadGrid, self.load_grid_cb)
        rospy.loginfo("All Done ...")
        rospy.spin()


    def save_grid_cb(self, req):
        print req
        print req.grid
        
        meta ={}
        msg_store = MessageStoreProxy(collection='strings') #change this
        msg_store.insert(req.grid,meta)

        return True

    def load_grid_cb(self, req):
        msg_store = MessageStoreProxy(collection='strings')
        query_meta={}
        sortq={'_meta.inserted_at':-1}
        message_list = msg_store.query(strands_exploration_msgs.msg.FremenGrid._type, {}, query_meta, sort_query= sortq.items())
        
#        for i in message_list:
#            print i

        number = req.number
        if number == -1:
            number = 0
        elif number >= len(message_list) :
            number = len(message_list)-1
        
        
        return message_list[number][0]


if __name__ == '__main__':
    rospy.init_node('grid_store')
    server = grid_store()