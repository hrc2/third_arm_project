#!/usr/bin/env python

# ik_node testing node

import numpy as np
from numpy import *
import rospy
import tf
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion


pub = rospy.Publisher('ik_cmd', Pose, queue_size=10)

linkLengths = [-0.08,0.045, 0.135] # [m]
linkConstraints = []

def control():

	# Intialize the node and name it
	rospy.init_node('ik_test', anonymous=True)


	testJ = [0, 0, 0, 0, 0]
	rospy.loginfo(testJ)
	newPose = jointToPose(testJ)
	pub.publish(newPose)

	rospy.spin()

def jointToPose(joint):

	FK = forwardKinematics(joint)
	quat = Quaternion(matrix=FK)

	newPose = Pose()

	newPose.position.x = FK[0][3]
	newPose.position.y = FK[1][3]
	newPose.position.z = FK[2][3]

	newPose.orientation.w = quat.q[0]
	newPose.orientation.x = quat.q[1]
	newPose.orientation.y = quat.q[2]
	newPose.orientation.z = quat.q[3]

	return newPose



def forwardKinematics(joint):
	# Performs forward kinematics on the third_arm using the Denavit-Hartenberg method

	FK = np.identity(4) # Identity matrix
	alphas = [pi/2, pi/2, 0, pi/2, pi/2]
	ais = np.array([0, linkLengths[0], 0, 0, linkLengths[2]])
	D = [0, 0, joint[2], linkLengths[1], 0]
	thetas = joint
	thetas[2] = pi # theta 3 is not a rotation

	for i in range(5):
		new_Transform = np.array([[cos(thetas[i]), -cos(alphas[i])*sin(thetas[i]), sin(alphas[i])*sin(thetas[i]), ais[i]*cos(thetas[i])],
                     [sin(thetas[i]), cos(alphas[i])*cos(thetas[i]), -sin(alphas[i])*cos(thetas[i]), ais[i]*sin(thetas[i])],
                     [0, sin(alphas[i]), cos(alphas[i]), D[i]],
                     [0, 0, 0, 1]])
		FK = np.dot(FK,new_Transform)

	return FK


if __name__ == '__main__':
    # Go to the main loop
    try:
        control()
    except rospy.ROSInterruptException:
        pass