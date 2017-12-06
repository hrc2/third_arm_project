#!/usr/bin/env python

# Function that takes in a universal transform matrix in the base frame and
# returns joint angles for the HRC2 third_arm
# Math by Vighnesh Vatsal
# Implementation by Kevin Kruempelstaedter

import numpy as np
from numpy import *
import time
import rospy
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion


# kinematic constraints of the arm
linkLengths = [-0.08,0.045, 0.135] # [m]
linkConstraints = []

#T = np.ones((4,4), dtype=float)

# test position is default arm position
#test1 = np.array([[0,0,0,0],[0,pi/2,0,0.54],[0,0,0,-0.25],[0,0,0,1]],dtype=np.float64)
# arm slightly turned to right 
#test2 = np.array([[0,0,0,0],[0,0,0,0.54],[0,0,0,-0.25],[0,0,0,1]],dtype=np.float64)
#print test1

#t_pred = np.zeros(5)

def callback(data):
	start_time = time.time()

	T = poseToMatrix(data)

	joint = getJointAngles(T)
	rospy.loginfo('==================Joint Angles===================')
	rospy.loginfo(joint)

	if isSingular(joint):
		rospy.loginfo('WARNING: Calculated IK solution is singular!')
	else:
		elapsed_time = time.time() - start_time
		rospy.loginfo('IK solution calculated in %s seconds' % elapsed_time)

	FK = forwardKinematics(joint)
	rospy.loginfo('Forward Kinematics')
	rospy.loginfo(FK)

def ros_control():
	# Intialize the node and name it
    rospy.init_node('ik_node', anonymous=True)
    rospy.loginfo('Node started')
    rospy.loginfo('listening to ik_cmd for pose commands')

    # pose command to be converted into joint space
    rospy.Subscriber("ik_cmd", Pose, callback)

    rospy.spin()

def poseToMatrix(pose):

	pos = pose.position
	quat = pose.orientation

	x = pos.x
	y = pos.y
	z = pos.z

	q = Quaternion(quat.w, quat.x, quat.y, quat.z)
	#M = quatToMatrix(quat.x, quat.y, quat.z, quat.w)

	T = q.transformation_matrix

	T[0][3] = x
	T[1][3] = y
	T[2][3] = z
	rospy.loginfo(T)
	return T.astype(np.float64)


def quatToMatrix(x, y, z, w):
	M = np.ones((3,3))

	# M[row][column]
	M[0][0] = 1 - 2*(2**y) - 2*(2**z)
	M[0][1] = 2*x*y + 2*w*z
	M[0][2] = z*x*z - 2*w*y

	M[1][0] = 2*x*y - 2*w*z
	M[1][1] = 1- 2*(2**x) - 2*(2**z)
	M[1][2] = 2*y*z + 2*w*z

	M[2][0] = 2*x*z + 2*w*y
	M[2][1] = 2*y*z - 2*w*x
	M[2][2] = 1 - 2*(2**x) - 2*(2**y)

	return M

def getJointAngles(T):
	linkLengths = [-0.08,0.045, 0.135]; # [m]
	t_pred = np.zeros(5)
	tVector = np.reshape(T,(1,16))
	goalVec = tVector[0]

	t_pred[0] = calcTheta1(goalVec,linkLengths[2])

	c4, t_pred[1] = calcTheta2(goalVec, t_pred[0])

	t_pred[3] = calcTheta4(goalVec, t_pred[1], c4)

	t_pred[4] = calcTheta5(goalVec, t_pred[1], c4)

	t_pred[2] = calcTheta3(goalVec, t_pred[1], t_pred[3], t_pred[4], linkLengths)

	return t_pred

def calcTheta1(vars, linkLength):
	r1y = vars[1]
	r1x = vars[0]
	py = vars[13]
	px = vars[12]

	theta1 = np.arctan2(py- linkLength*r1y, px - linkLength*r1x)
	return theta1

def reCalcTheta1(vars, theta2, c4):
	r2x = vars[4]
	r2y = vars[5]
	r2z = vars[6]

	A = np.array([[c4, cot(theta2)*r2z], [cot(theta2)*r2z, -c4]])
	b = np.array([r2x,r2y])

	avec = np.linalg.solve(A,b)

	theta1 = np.arctan2(avec[0],avec[1])
	return theta1

def calcTheta2(vars, theta1):
	r2x = vars[4]
	r2y = vars[5]
	r2z = vars[6]

	A = np.array([[sin(theta1), cos(theta1)*r2z],[-cos(theta1), sin(theta1)*r2z]],dtype=np.float64)
	b = np.array([[r2x],[r2y]])	
	#sol1, sol2 = np.linalg.lstsq(A,b)[0]
	sol = np.linalg.solve(A,b)
	sol1 = sol[0]
	sol2 = sol[1]
	print 'solution 2 %s' %sol2
	# This also gives us cos(theta4)
	c4 = sol1

	# check range: [0,pi/2]
	#candidate = np.arctan2(1,sol2)\
	candidate = sol2
	if candidate >0 and candidate <pi/2:
		theta2 = candidate
	elif candidate>pi/2 and candidate <= pi:
		theta2 = -candidate + pi
	elif candidate<-pi/2 and candidate >-pi:
		theta2 = pi+ candidate
	else:
		theta2 = -candidate
	return c4, theta2

def calcTheta3(vars, theta2, theta4, theta5, links):
	# Calculate joint variable 3 = extension length
	# range [0.33, 0.45]

	pz = vars[14]
	l1 = links[0]
	l2 = links[1]
	l3 = links[2]

	extension = (-pz + l1 - l3*cos(theta2)*sin(theta5) - l3*cos(theta4)*cos(theta5)*sin(theta2) - l2*cos(theta2))/cos(theta2)
	return extension

def calcTheta4(vars, theta2, c4):
	# range [-pi,pi]

	r2z = vars[6]
	s4 = -r2z/sin(theta2)
	theta4 = np.arctan2(s4,c4)
	return theta4

def calcTheta5(vars, theta2, c4):
	# range [0,pi]

	r1z = vars[2]
	r3z = vars[10]

	A = np.array([[-cos(theta2), -c4*sin(theta2)],[-c4*sin(theta2), cos(theta2)]])
	sol = np.linalg.solve(A,np.array([[r1z],[r3z]]))

	theta5 = np.arctan2(sol[0],sol[1])

	return theta5

def isSingular(joint):
	# checks if a set of joint parameters are singular or not
	# Returns True if joint values lead to a singular FK solution

	T = forwardKinematics(joint)

	if (np.linalg.matrix_rank(T)== 4):
		return False
	else:
		return True


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




#getJointAngles(test1)

if __name__ == '__main__':
    # Go to the main loop
    try:
        ros_control()
    except rospy.ROSInterruptException:
        pass