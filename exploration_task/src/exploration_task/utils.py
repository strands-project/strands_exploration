#!/usr/bin/env python

import rospy
import actionlib
import flir_pantilt_d46.msg
from sensor_msgs.msg import JointState
import strands_gazing.msg


class Gaze():
    def __init__(self):
        self.people_closest_topic = '/upper_body_detector/closest_bounding_box_centre'
        # Gaze client
        rospy.loginfo("Creating gaze client")
        self.gazeClient = actionlib.SimpleActionClient(
            'gaze_at_pose',
            strands_gazing.msg.GazeAtPoseAction
        )
        self.gazeClient.wait_for_server()
        rospy.loginfo("...done")

    def people(self):
        goal = strands_gazing.msg.GazeAtPoseGoal
        goal.topic_name = self.people_closest_topic
        goal.runtime_sec = 0
        self.gazeClient.send_goal(goal)

    def preempt(self):
        self.gazeClient.cancel_all_goals()


class PTU():
    def __init__(self):
        # PTU client
        rospy.loginfo("Creating PTU client")
        self.ptuClient = actionlib.SimpleActionClient(
            'SetPTUState',
            flir_pantilt_d46.msg.PtuGotoAction
        )
        self.ptuClient.wait_for_server()
        rospy.loginfo("...done")

    def turnPTU(self, pan, tilt):
        goal = flir_pantilt_d46.msg.PtuGotoGoal()
        goal.pan = pan
        goal.tilt = tilt
        goal.pan_vel = 60
        goal.tilt_vel = 60
        self.ptuClient.send_goal(goal)


class Head():
    def __init__(self):
        self.pub = rospy.Publisher('/head/commanded_state', JointState, queue_size=10)

    def resetHead(self):
        self.head_command = JointState()
        self.head_command.name = ["HeadPan", "HeadTilt", "EyesTilt", "EyesPan", "EyeLids"]
        self.head_command.position = [0, 0, 0, 0, 100]
        self.pub.publish(self.head_command)

    def turnHead(self):
        self.head_command = JointState()
        self.head_command.name = ["HeadPan", "HeadTilt", "EyesTilt", "EyesPan", "EyeLids"]
        self.head_command.position = [-180, 0, 0, 0, 100]
        self.pub.publish(self.head_command)

    def closeEyes(self):
        self.head_command = JointState()
        self.head_command.name = ["EyeLids"]
        self.head_command.position = [0]
        self.pub.publish(self.head_command)

    def openEyes(self):
        self.head_command = JointState()
        self.head_command.name = ["EyeLids"]
        self.head_command.position = [100]
        self.pub.publish(self.head_command)
