#!/usr/bin/env python  
import rospy
import roslib
import random
import math
import arbotix_msgs.srv
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import JointState
import numpy as np

j = 0

def callback(data):
	pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)
	pub_human = rospy.Publisher('/joint_states', JointState, queue_size=1)
	

	state = data

	val1 = 0
	val2 = 0
	val3 = 0
	val4 = 0
	val5 = 0.8

	state = data
	h_len = len(data.name)
	h_state = data
	h_state.position = [0.0 for i in range(h_len)]
	hval1 = 0
	hval2 = 1.57
	hval3 = 0
	hval4 = 0
	num = 50

	#First movement: Outwards
	val1 = np.linspace(0,-1.42,num)
	val2 = np.linspace(0,-0.68,num)
	val3 = np.linspace(0,0.15,num)

	for i in range(num):
		v1 = val1[i]
		v2 = val2[i]
		v3 = val3[i]
		h_state.position[12:15] = [hval1,hval2,hval3,hval4]
		h_state.position[0:5] = [v1,v2,v3,val4,val5,-val5]
		state.position = [v1,v2,v3,val4,val5,-val5]
		pub.publish(state)
		pub_human.publish(h_state)

	#Second movement: Gripper on
	val5 = 0.1
	h_state.position[12:15] = [hval1,hval2,hval3,hval4]
	h_state.position[0:5] = [v1,v2,v3,val4,val5,-val5]
	state.position = [v1,v2,v3,val4,val5,-val5]
	pub.publish(state)
	pub_human.publish(h_state)

	#Third movement: Inwards
	val1 = np.linspace(v1,1.00,num)
	val2 = np.linspace(v2,0.22,num)
	val3 = np.linspace(v3,0.0,num)

	for i in range(num):
		v1 = val1[i]
		v2 = val2[i]
		v3 = val3[i]
		h_state.position[12:15] = [hval1,hval2,hval3,hval4]
		h_state.position[0:5] = [v1,v2,v3,val4,val5,-val5]
		state.position = [v1,v2,v3,val4,val5,-val5]
		pub.publish(state)
		pub_human.publish(h_state)

	#Fourth movement: grip open 
	val5 = 0.8
	h_state.position[12:15] = [hval1,hval2,hval3,hval4]
	h_state.position[0:5] = [v1,v2,v3,val4,val5,-val5]
	state.position = [v1,v2,v3,val4,val5,-val5]
	pub.publish(state)
	pub_human.publish(h_state)	

if __name__ == '__main__':
	rospy.init_node('publish_states_example_traj')

	pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)
	pub_human = rospy.Publisher('/joint_states', JointState, queue_size=1)

	rospy.Subscriber('/joint_states', JointState, callback)

	#rate = rospy.Rate(5)
	#while not rospy.is_shutdown():


	
	
	
	#rospy.Subscriber('/joint_states', JointState, callback)
		
	rospy.spin()
	#rate.sleep()
	