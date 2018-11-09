#!/usr/bin/env python
import rospy

import math
import time
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
import dynamixel_controllers.srv
from diagnostic_msgs.msg import DiagnosticArray
from apriltags_ros.msg import AprilTagDetectionArray
import csv

class ik_2d_apriltags:    

    currval = [0.0, 0.0]
    command = [0.0, 0.0]


    def __init__(self):
        rospy.init_node('thirdarm_apriltags_2d')

        self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command', Float64, queue_size=1)
        #self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command', Float64, queue_size=1)
        #self.pub_motor4 = rospy.Publisher('/wrist_controller/command', Float64, queue_size=1)
        #self.pub_motor5 = rospy.Publisher('/wrist_tilt_controller/command', Float64, queue_size=1)
        #self.pub_motor6 = rospy.Publisher('/gripper_controller/command', Float64, queue_size=1)

        self.set_speed1 = rospy.ServiceProxy('base_swivel_controller/set_speed', dynamixel_controllers.srv.SetSpeed)
        self.set_speed3 = rospy.ServiceProxy('arm_extension_controller/set_speed', dynamixel_controllers.srv.SetSpeed)
        self.trans = tf.TransformListener()

        #self.pubvec = [self.pub_motor1, self.pub_motor2, self.pub_motor3, self.pub_motor4, self.pub_motor5, self.pub_motor6]

        #self.js = Joy

    def check(self,ax,bu):
		ch = 0
		if np.count_nonzero(np.asarray(ax)) >= 1:
			ch = 1
		elif np.count_nonzero(np.asarray(bu)) >= 1:
			ch = 1
		else:
			ch = 0

		#print ("Check: " + str(ch))
		return ch

    def positions_update(self, data):		
        (pos,rot) = self.trans.lookupTransform('/base_frame','/camera_rgb_frame', rospy.Time())
        #print("Base: " + str(pos))# + " , " + str(rot))
        self.base_x = data.detections[0].pose.pose.position.x
        self.base_y = data.detections[0].pose.pose.position.y
        self.base_z = data.detections[0].pose.pose.position.z
        #self.ee_x = data.detections[1].pose.pose.position.x
        #self.ee_y = data.detections[1].pose.pose.position.y
        print("Base- X: " + str(self.base_x) + " Y: " + str(self.base_y) + " Z: " + str(self.base_z))
        #print("Gripper- X: " + str(self.ee_x) + " Y: " + str(self.ee_y))

    def pos_msg_update(self):
	data = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
	self.base_x = data.detections[0].pose.pose.position.x
	self.base_y = data.detections[0].pose.pose.position.y
	#self.ee_x = data.detections[1].pose.pose.position.x
	#self.ee_y = data.detections[1].pose.pose.position.y
        print("Base- X: " + str(self.base_x) + " Y: " + str(self.base_y))
        #print("Gripper- X: " + str(self.ee_x) + " Y: " + str(self.ee_y))

    def positions_display(self):
    	print("Base- X: " + str(self.base_x) + " Y: " + str(self.base_y))
    	print("Gripper- X: " + str(self.ee_x) + " Y: " + str(self.ee_y))	

    def run(self):
    	self.tags = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
    	print("Detected Initial States: ")
    	time.sleep(1)
    	self.rate = rospy.Rate(1)
    	if self.tags.detections[0].pose.pose.position.x != 0 and self.tags.detections[1].pose.pose.position.x != 0:
    		self.base_x0 = self.tags.detections[0].pose.pose.position.x
    		self.base_y0 = self.tags.detections[0].pose.pose.position.y
    		self.ee_x0 = self.tags.detections[1].pose.pose.position.x
    		self.ee_y0 = self.tags.detections[1].pose.pose.position.y

    		print("Base Intials - X: " + str(self.base_x0) + " Y: " + str(self.base_y0))
    		print("Gripper Intials - X: " + str(self.ee_x0) + " Y: " + str(self.ee_y0))
    		time.sleep(1)

		while not rospy.is_shutdown():
		    	self.pos_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.positions_update)
			self.rate.sleep()
			#self.pos_msg_update()
		    	#self.positions_display()



if __name__ == '__main__':
    t1 = ik_2d_apriltags()
    t1.run()
