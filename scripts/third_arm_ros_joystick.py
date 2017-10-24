#!/usr/bin/env python
import rospy

import math
import numpy as np
import sys
import tf
import PyKDL
import dynamic_reconfigure.server
import argparse

import sensor_msgs.msg
from sensor_msgs.msg import Imu, JointState, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64

import random
import arbotix_msgs.srv
import dynamixel_msgs.msg
from diagnostic_msgs.msg import DiagnosticArray
import csv

class thirdarm_joystick:    

    currval = [0.0, 0.0, 0.0, 0.0, 0.0]
    command = [0.0, 0.0, 0.0, 0.0, 0.0]


    def __init__(self):
        rospy.init_node('thirdarm_joystick')

        self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command', Float64, queue_size=1)
        self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command', Float64, queue_size=1)
        self.pub_motor4 = rospy.Publisher('/wrist_controller/command', Float64, queue_size=1)
        self.pub_motor5 = rospy.Publisher('/gripper_controller/command', Float64, queue_size=1)

        self.pubvec = [self.pub_motor1, self.pub_motor2, self.pub_motor3, self.pub_motor4, self.pub_motor5]


    def run(self):
   		rospy.Subscriber("joy", Joy, self.get_joystick)
   		rospy.spin()
        
    def get_current_state(self):
        topic_list = ['/base_swivel_controller/state' , '/vertical_tilt_controller/state' , '/arm_extension_controller/state' , '/wrist_controller/state' , '/gripper_controller/state']
        print("Waiting for joint states")
        self.count = 0
        for topic in topic_list:
            print("Topic name : " + topic)
            data = rospy.wait_for_message(topic, dynamixel_msgs.msg.JointState)            
            self.currval[self.count] = data.current_pos
            self.count += 1
        print("JS data: " + str(self.currval))

    def get_joystick(self, js):

    	self.get_current_state()

    	if js.buttons(0) > 0.1:
    		grip_state = +1
    	elif js.buttons(1) > 0.1:
    		grip_state = -1

    	cvec = [js.axes(4) , js.axes(3) , js.axes(1) , js.axes(0) , grip_state]

    	cmd_scale = [-0.3, 0.5, 0.6, -0.5, 0.3]

    	for i in range(len(self.command)):
    		self.command[i] = self.currval[i] + cmd_scale[i]*np.sign(cvec[i])

    	print("Command Values are " + str(self.command))        
        
        for i in range(len(self.pubvec)):
       		self.pubvec[i].publish(self.command[i])

	def test_joystick(self, js):

		self.get_current_state()

		if js.buttons(0) > 0.1:
			grip_state = +1
		elif js.buttons(1) > 0.1:
			grip_state = -1

		cvec = [js.axes(4) , js.axes(3) , js.axes(1) , js.axes(0) , grip_state]

		cmd_scale = [-0.3, 0.5, 0.6, -0.5, 0.3]

		for i in range(len(self.command)):
			self.command[i] = self.currval[i] + cmd_scale[i]*np.sign(cvec[i])

		print("Command Values are " + str(self.command))        
	        
        # for i in range(len(self.pubvec)):
        #    self.pubvec[i].publish(self.command[i])







if __name__ == '__main__':
    t1 = thirdarm_joystick()
    t1.run()
