#!/usr/bin/env python
# Read the end-effector state from simulation, get 

import numpy as np
from numpy import *
import time
import rospy
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion
import tf
import math
import ros_ik

class thirdArm5dof:

	def __init__(self):
		rospy.init_node('thirdArm5dof')
		self.listener = tf.TransformListener()

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

	def read_tf(self):
		while not rospy.is_shutdown():
			try:
				(self.trans, self.rot) = self.listener.lookupTransform('gripper_end_point', 'base_motor', rospy.Time(0))
				self.pose_data = self.trans_to_pose(self.trans, self.rot)
				
				#rospy.loginfo('Transform between end-effector and moving base: {0} , {1}'.format(self.trans, self.rot))
				self.joint_angles = ros_ik.IK(self.pose_data)
				rospy.loginfo('Joint angles from IK are: {0} '.format(self.joint_angles))
   			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
   				continue


if __name__ == '__main__':
    
    try:
        td = thirdArm5dof()
        td.read_tf()
    except rospy.ROSInterruptException:
        pass