#!/usr/bin/env python

import rospy
from periodic_poisson_processes.poisson_processes import PeriodicPoissonProcesses


class PoissonWrapper(object):

    def __init__(
        self, topic, topic_msg, topic_attribute, value,
        window=10, increment=1, periodic_cycle=10080
    ):
        rospy.loginfo("Initializing poisson wrapper...")
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        self.time_window = window
        self.time_increment = increment
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        self.process = PeriodicPoissonProcesses(window, increment, periodic_cycle)
        rospy.loginfo(
            "Listening to topic %s with type %s, counting attribute %s with value %s" %
            (topic, topic_msg._type, topic_attribute, str(value))
        )
        self.attribute = topic_attribute
        self.value = value
        self.topic = topic
        self.topic_msg = topic_msg
        self.load_from_db()
        rospy.Subscriber(topic, topic_msg, self._cb, None, 10)

    def _cb(self, msg):
        current_time = rospy.Time.now()
        value = getattr(msg, self.attribute)
        count = 0
        if isinstance(value, (list, tuple)):
            count = sum([1 for i in value if value == self.value])
        elif self.value == value:
            count = 1
        self.process.update(current_time, count)
        self._store(current_time)

    def _store(self, start_time):
        meta = {
            "type": self.topic_msg._type, "topic": self.topic,
            "recorded_value": self.value, "recorded_attribute": self.attribute
        }
        self.process._store(start_time, meta)

    def retrieve_from_to(self, start_time, end_time):
        return self.process.retrieve(start_time, end_time)

    def load_from_db(self):
        meta = {
            "type": self.topic_msg._type, "topic": self.topic,
            "recorded_value": self.value, "recorded_attribute": self.attribute
        }
        self.process.retrieve_from_mongo(meta)

    def store_to_db(self):
        meta = {
            "type": self.topic_msg._type, "topic": self.topic,
            "recorded_value": self.value, "recorded_attribute": self.attribute
        }
        self.process.store_to_mongo(meta)
