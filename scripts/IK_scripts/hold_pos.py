#!/usr/bin/env python
# Read the end-effector state from simulation, move the joints to hold position

import numpy as np
from numpy import *
import time
import rospy
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion
from sensor_msgs.msg import JointState
import tf
import math
import ros_ik
import random
import time
from visualization_msgs.msg import Marker
import csv
#import base_gen

class thirdArm5dof:

	def __init__(self):
		rospy.init_node('thirdArm5dof')
		self.listener = tf.TransformListener()
		self.js_pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)		

	def trans_to_pose(self, trans, rot):
		pose_data = Pose()
		
		pose_data.position.x = trans[0]
		pose_data.position.y = trans[1]
		pose_data.position.z = trans[2]

		pose_data.orientation.x = rot[0]
		pose_data.orientation.y = rot[1]
		pose_data.orientation.z = rot[2]
		pose_data.orientation.w = rot[3]

		return pose_data

	def write_to_file(self, data):
		with open(self.fname,'a') as f:						
			writer = csv.writer(f, quoting=csv.QUOTE_ALL)			
			writer.writerow([data])

	def read_tf(self):
		while not rospy.is_shutdown():
			try:
				self.listener.waitForTransform('base_link', 'gripper_end_point', rospy.Time(), rospy.Duration(0.01))
				(self.trans, self.rot) = self.listener.lookupTransform('base_link', 'gripper_end_point', rospy.Time())
				self.pose_data = self.trans_to_pose(self.trans, self.rot)				
				#rospy.loginfo('Transform between end-effector and moving base: {0} , {1}'.format(self.trans, self.rot))
				self.joint_angles = ros_ik.IK(self.pose_data)
				#rospy.loginfo('Joint angles from IK are: {0} '.format(self.joint_angles))
   			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
   				continue

   	def make_marker(self,trans,rgba):
		marker = Marker()
		marker.header.frame_id = "environment_x"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.05
		marker.scale.y = 0.05
		marker.scale.z = 0.05
		marker.color.a = rgba[3]
		marker.color.r = rgba[0]
		marker.color.g = rgba[1]
		marker.color.b = rgba[2]
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = trans[0]
		marker.pose.position.y = trans[1] 
		marker.pose.position.z = trans[2] 
		marker.lifetime = rospy.Duration()
		return marker

   	def initialize(self):
   		self.js = rospy.wait_for_message('/joint_states', JointState)
   		self.fname = 'ik_hold_pos_test_'+str(int(time.mktime(time.localtime())))+'.csv'
   		# first three are environment values: set to zero
   		dof = [0, 0, 0]
   		dof.append(0.3*(-math.pi + 2*math.pi*random.random()))
   		dof.append(math.pi/6 + (math.pi/3)*random.random())
   		dof.append(0.11*random.random())
   		dof.append((-math.pi + 2*math.pi*random.random()))
   		dof.append(math.pi*random.random())
   		   		
   		self.js.position = dof
   		rospy.loginfo(self.js)   		
   		
   		self.js.header.stamp = rospy.Time.now()   		
   		self.js_pub.publish(self.js)
   		self.listener.waitForTransform('base_link', 'gripper_end_point', rospy.Time(), rospy.Duration(0.5))
   		(lin, ang) = self.listener.lookupTransform('base_link', 'gripper_end_point', rospy.Time())
   		self.orig_pos = lin
   		topic = 'init_marker'
		marker_pub = rospy.Publisher(topic, Marker, queue_size=1)
		rgb = [0.0,1.0,0.0,0.5]
		marker = self.make_marker(self.orig_pos,rgb)
		marker_pub.publish(marker)
		self.write_to_file(self.js.position)

   		
   	def traj_start(self):
   		traj_flag = rospy.Publisher('/traj_flag', int ,queue_size=1)
   		traj_flag.publish(1)

   	def traj_gen(self):
   		t = time.time()
   		scale = 0.01
   		xt = 0.01*cos(1.5*t)*cos(t)
   		yt = 0.01*cos(1.5*t)*sin(t)
   		zt = 0.01*sin(1.5*t)
   		old_pos = self.js.position
   		self.new_pos = [xt, yt, zt, old_pos[3], old_pos[4], old_pos[5], old_pos[6], old_pos[7]]
   		self.js.position = self.new_pos
   		self.js_pub.publish(self.js)

   	def solve_ik(self):
   		self.listener.waitForTransform('base_link', 'gripper_end_point', rospy.Time(), rospy.Duration(0.5))
   		(self.trans, self.rot) = self.listener.lookupTransform('base_link', 'gripper_end_point', rospy.Time())
   		env_change = self.new_pos[0:3]
   		new_trans = [0, 0, 0]
   		for i in range(3):
   			new_trans[i] = self.orig_pos[i] - env_change[i]
		self.pose_data = self.trans_to_pose(new_trans, self.rot)
		self.joint_angles = ros_ik.IK(self.pose_data)
		self.new_pos[3:] = self.joint_angles
		self.js.position = self.new_pos
		self.js_pub.publish(self.js)
		self.write_to_file(self.js.position)


   	def start(self):
   		# Move third arm to random pose in mid joint-space, hold end-effector at that pose
   		self.initialize()
   		rate = rospy.Rate(2)
   		while not rospy.is_shutdown():
   			#rate.sleep()
   			# Generating random trajectory for the base link
   			self.traj_gen()
   			rate.sleep()
   			# Moving arm to new position to compensate
   			self.solve_ik()
   			rate.sleep()
   		


if __name__ == '__main__':
    
    try:
        td = thirdArm5dof()
        td.start()
    except rospy.ROSInterruptException:
        pass