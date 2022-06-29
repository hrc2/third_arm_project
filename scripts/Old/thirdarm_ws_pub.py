#!/usr/bin/env python  
import rospy
import roslib
import random
import math
import arbotix_msgs.srv
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import JointState
i = 0
def callback(data):
	pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)
	global i
	i += 1
	state = data
	val1 = -1.57 + 3.14*random.random()
	val2 = -1.57 + (30*math.pi/180 + 1.57)*random.random()
	val3 = 0.15*random.random()
	if i<100:
		state.position = [val1,val2,val3,val1,val1,val1]
	
	# if i < 1.57 and incr == 1:
	# 	i += 0.01
	# elif incr == -1:
	# 	i -= 0.01

	# if i>1.57:
	# 	incr = -1
	# elif i<0:
	# 	incr = 1

	pub.publish(state)

if __name__ == '__main__':
	rospy.init_node('publish_states_thirdarm_rviz')
	#rate = rospy.Rate(5)
	#while not rospy.is_shutdown():
	rospy.Subscriber('/joint_states', JointState, callback)
	rospy.spin()
	#rate.sleep()
	