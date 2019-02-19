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

    #currval = [0.0, 0.0]
    #command = [0.0, 0.0]


    def __init__(self):
        rospy.init_node('apriltags_2d_poser')
        self.pub_base_pose = rospy.Publisher('/base_pose', Point, queue_size=1)
        self.pub_ee_pose = rospy.Publisher('/ee_pose', Point, queue_size=1)
        


    def positions_update(self, data):		
        #(pos,rot) = self.trans.lookupTransform('/base_frame','/camera_rgb_frame', rospy.Time())
        #print("Base: " + str(pos))# + " , " + str(rot))
	try:
	        self.base_x = data.detections[0].pose.pose.position.x
	        self.base_y = data.detections[0].pose.pose.position.y
	        self.base_z = data.detections[0].pose.pose.position.z
	        self.ee_x = data.detections[1].pose.pose.position.x
	        self.ee_y = data.detections[1].pose.pose.position.y
	        self.ee_z = data.detections[1].pose.pose.position.z

		self.publish_poses(data.detections[0].pose.pose.position, data.detections[1].pose.pose.position)

	except (NameError, IndexError):
		return
        #print("Base- X: " + str(self.base_x) + " Y: " + str(self.base_y) + " Z: " + str(self.base_z))
        #print("Gripper- X: " + str(self.ee_x) + " Y: " + str(self.ee_y) + " Z: " + str(self.ee_z))

   	

    def publish_poses(self, base_pose, ee_pose):
	self.pub_base_pose.publish(base_pose)
	self.pub_ee_pose.publish(ee_pose)


 #    def pos_msg_update(self):
	# data = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
	# self.base_x = data.detections[0].pose.pose.position.x
	# self.base_y = data.detections[0].pose.pose.position.y
	# self.ee_x = data.detections[1].pose.pose.position.x
	# self.ee_y = data.detections[1].pose.pose.position.y
 #        print("Base- X: " + str(self.base_x) + " Y: " + str(self.base_y))
 #        print("Gripper- X: " + str(self.ee_x) + " Y: " + str(self.ee_y))

    # def positions_display(self):
    # 	print("Base- X: " + str(self.base_x) + " Y: " + str(self.base_y))
    # 	print("Gripper- X: " + str(self.ee_x) + " Y: " + str(self.ee_y))	

    def run(self):
	flag1 = 0
	flag2 = 0
    	while(1-flag1*flag2):
    		self.tags = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
    		try:
			self.tags.detections[0].pose.pose.position.x
			flag1 = 1
			#break
		except (NameError, IndexError):
			continue
		try:
			self.tags.detections[1].pose.pose.position.x
			flag2 = 1
			#break
		except (NameError, IndexError):
			continue

    	print("Detected Initial States: ")
    	
    	time.sleep(1)
    	
    	#self.rate = rospy.Rate(1)
    	
    	if self.tags.detections[0].pose.pose.position.x != 0 and self.tags.detections[1].pose.pose.position.x != 0:
    		self.base_x0 = self.tags.detections[0].pose.pose.position.x
    		self.base_y0 = self.tags.detections[0].pose.pose.position.y
    		self.ee_x0 = self.tags.detections[1].pose.pose.position.x
    		self.ee_y0 = self.tags.detections[1].pose.pose.position.y

    		print("Base Intials - X: " + str(self.base_x0) + " Y: " + str(self.base_y0))
    		print("Gripper Intials - X: " + str(self.ee_x0) + " Y: " + str(self.ee_y0))
    		time.sleep(1)

		#while not rospy.is_shutdown():
    	self.pos_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.positions_update)
	rospy.spin()
			#self.rate.sleep()
			#self.pos_msg_update()
		    	#self.positions_display()



if __name__ == '__main__':
    t1 = ik_2d_apriltags()
    t1.run()