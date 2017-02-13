#!/usr/bin/env python  
import rospy
import roslib
import random
import math
import arbotix_msgs.srv
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import JointState

j = 0

def callback(data):
	pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)
	pub_human = rospy.Publisher('/joint_states', JointState, queue_size=1)
	
	global j
	j += 1
	if j<math.pow(10,9):
		state = data
		val1 = -3.14 + 6.28*random.betavariate(1, 1)
		val2 = -60*math.pi/180 + (120*math.pi/180)*random.betavariate(0.433, 0.433)
		val3 = 0.15*random.betavariate(0.4, 0.4)
		val4 = 1.0
		#if i<100:
		state.position = [val2,val3,val4]
		
		h_len = len(data.name)
		h_state = data
		h_state.position = [0.0 for i in range(h_len)]
		h_state.position[0:3] = [val2,val3,val4,val4]#state.position

		hval1 = -1.57 + 3.14*random.betavariate(0.6, 0.6)
		hval2 = -1.57 + 3.927*random.betavariate(0.55, 0.55)
		hval3 = -1.57 + 3.14*random.betavariate(0.6, 0.6)
		hval4 = 1.57*random.betavariate(0.5, 0.5)
		h_state.position[10:13] = [hval1,hval2,hval3,hval4]

		pub.publish(state)
		pub_human.publish(h_state)
	
	

if __name__ == '__main__':
	rospy.init_node('v1_publish_states_thirdarm_rviz')
	#rate = rospy.Rate(5)
	#while not rospy.is_shutdown():
	
	
	rospy.Subscriber('/joint_states', JointState, callback)
		
	rospy.spin()
	#rate.sleep()
	