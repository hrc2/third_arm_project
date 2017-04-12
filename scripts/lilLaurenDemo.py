#!/usr/bin/env python  
import rospy
import roslib
import random
import math
import arbotix_msgs.srv
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import JointState
import dynamixel_msgs.msg
import numpy as np
import csv

class ThirdArm:	

	h_names = ['base_to_motor1', 'swivel_to_motor2', 'extension', 'wrist_motor_to_extension', 'finger1_to_gripper_motor', 'finger2_to_gripper_motor', 'spine_0', 'spine_1', 'spine_2', 'neck_0', 'neck_1', 'neck_2', 'right_shoulder_0', 'right_shoulder_1', 'right_shoulder_2', 'right_elbow_0', 'right_wrist_0', 'right_wrist_1', 'right_wrist_2', 'left_shoulder_0', 'left_shoulder_1', 'left_shoulder_2', 'left_elbow_0', 'left_wrist_0', 'left_wrist_1', 'left_wrist_2', 'right_hip_0', 'right_hip_1', 'right_hip_2', 'right_knee_0', 'right_ankle_0', 'right_ankle_1', 'left_hip_0', 'left_hip_1', 'left_hip_2', 'left_knee_0', 'left_ankle_0', 'left_ankle_1']
	arm_names = ['base_to_motor1', 'swivel_to_motor2', 'extension', 'wrist_motor_to_extension', 'finger1_to_gripper_motor', 'finger2_to_gripper_motor']
	dofvals = []
	hvals = []
	torque = [0.0 for i in range(5)]
	h_state = JointState([],h_names,hvals,[],[])
	h_len = len(h_names)
	state = JointState([],h_names,dofvals,[],[])

	move = 1
	i = 0

	def push_data(self):

		if self.move == 1:
			self.set_data1()
		elif self.move == 2:
			self.set_data2()
		# elif self.move == 3:
		# 	self.set_data3()
		# elif self.move == 4:
		# 	self.set_data4()
		
		rospy.loginfo('To be pubbed: {0} and counter: {1}'.format(self.h_state.position,self.i))

		#self.pub_human.publish(self.h_state)
		#self.pub.publish(self.state)

		self.send_motor()

	def send_motor(self):
	# base_val = -self.h_state.position[0]
		# tilt_val = -0.3 + (-self.h_state.position[1])
		# extend_val = -0.3 + (-2.0+0.3)*self.h_state.position[2]/0.15
		# gripper_val =  1.3 - self.h_state.position[4]
		base_val = -self.dofvals[0]
		tilt_val = -0.3 + (-self.dofvals[1])
		extend_val = -0.3 + (-2.0+0.3)*self.dofvals[2]/0.15
		gripper_val =  1.5 - self.dofvals[4]
		self.pub_motor5.publish(gripper_val)
		self.pub_motor3.publish(extend_val)
		self.pub_motor2.publish(tilt_val)
		#tilt_val
		self.pub_motor1.publish(base_val)
		self.pub_motor4.publish(0.0)

	def print_torque(self,data):
		for j in range(2):
			self.torque[j] = data.motor_states[j].load

		

		write_data = self.torque
		with open('demo_task_3.csv','a') as f:
		 	writer = csv.writer(f,quoting=csv.QUOTE_ALL)
		 	writer.writerow(write_data)
		 	rospy.loginfo('\nTorque is: {0} \n'.format(write_data))	
		
	def set_data1(self):
		self.h_state.position = [0.0 for i in range(self.h_len)]
		i = self.i
		hval1 = 0
		hval2 = 1.57
		hval3 = 0
		hval4 = 0
		num = 50

		val1 = 0
		val2 = 0
		val3 = 0
		val4 = 0
		val5 = 1.3

		#First movement: Inwards
		val1 = np.linspace(0,-0.50,num)
		val2 = np.linspace(0,-0.05,num)
		val3 = np.linspace(0,0.01,num)


		v1 = val1[i]
		v2 = val2[i]
		v3 = val3[i]
		self.dofvals = [v1,v2,v3,val4,val5,-val5]
		self.hvals = [hval1,hval2,hval3,hval4]
		#self.state.position = self.dofvals
		self.h_state.position[0:6] = [v1,v2,v3,val4,val5,-val5]
		self.h_state.position[13] = 1.57
		rospy.loginfo('H_state pos is: {0}'.format(self.h_state.position))
		

		self.i = self.i + 1

		if self.i>=num:
			self.move = 2
			self.i = 0

	def set_data2(self):
		#Second movement: Gripper on
		self.h_state.position = [0.0 for i in range(self.h_len)]
		self.dofvals[4:6] = [0.05,-0.05]

		hval1 = 0
		hval2 = 1.57
		hval3 = 0
		hval4 = 0
		self.hvals = [hval1,hval2,hval3,hval4]
		self.h_state.position[12:16] = self.hvals
		self.h_state.position[0:6] = self.dofvals
		#self.state.position = self.dofvals

		#self.move = 3

	def set_data3(self):
		self.h_state.position = [0.0 for i in range(self.h_len)]
		num = 50
		i = self.i

		#Third movement: Outwards
		val1 = np.linspace(1.00,-3.00,num)
		val2 = np.linspace(0.1,0.3,num)
		val3 = np.linspace(0.15,0.0,num)
			
		
		v1 = val1[i]
		v2 = val2[i]
		v3 = val3[i]
		self.dofvals[0:3] = [v1,v2,v3]
		hval1 = 0
		hval2 = 1.57
		hval3 = 0
		hval4 = 0
		self.hvals = [hval1,hval2,hval3,hval4]
		self.h_state.position[12:16] = self.hvals
		self.h_state.position[0:6] = self.dofvals
		#self.state.position = self.dofvals

		self.i = self.i + 1
		
		if self.i>=num:
			self.move = 4

	def set_data4(self):
	#Fourth movement: grip open 
		self.h_state.position = [0.0 for i in range(self.h_len)]
		self.dofvals[4:6] = [0.8,-0.8]
		hval1 = 0
		hval2 = 1.57
		hval3 = 0
		hval4 = 0
		self.hvals = [hval1,hval2,hval3,hval4]
		self.h_state.position[12:16] = self.hvals
		self.h_state.position[0:6] = self.dofvals
		#self.state.position = self.dofvals

	def sub_once(self,data):
		self.state = data
		self.h_len = len(data.name)
		self.h_state = data
		rospy.loginfo('I subbed:{0}'.format(self.h_len))

		self.sub.unregister()

	def pubsub(self):
		self.rate = rospy.Rate(15)

		self.pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)
		self.pub_human = rospy.Publisher('/joint_states', JointState, queue_size=1)
		self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command',Float64, queue_size=1)
		self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command',Float64, queue_size=1)
		self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command',Float64, queue_size=1)
		self.pub_motor4 = rospy.Publisher('/wrist_controller/command',Float64, queue_size=1)
		self.pub_motor5 = rospy.Publisher('/gripper_controller/command',Float64, queue_size=1)

		while not rospy.is_shutdown():
			self.sub = rospy.Subscriber('/joint_states', JointState, self.sub_once)
			#self.motor_sub = rospy.Subscriber('/gripper_controller/state', dynamixel_msgs.msg.JointState, self.get_torque)
			
			self.push_data()
			self.rate.sleep()
			self.motor_sub = rospy.Subscriber('/motor_states/third_arm_port', dynamixel_msgs.msg.MotorStateList, self.print_torque)
		
		#rospy.spin()
			




# def callback(data):
# 	pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)
# 	pub_human = rospy.Publisher('/joint_states', JointState, queue_size=1)
	

# 	state = data

# 	val1 = 0
# 	val2 = 0
# 	val3 = 0
# 	val4 = 0
# 	val5 = 0.8

# 	state = data
# 	h_len = len(data.name)
# 	h_state = data
# 	h_state.position = [0.0 for i in range(h_len)]
# 	hval1 = 0
# 	hval2 = 1.57
# 	hval3 = 0
# 	hval4 = 0
# 	num = 50

# 	#First movement: Outwards
# 	val1 = np.linspace(0,-1.42,num)
# 	val2 = np.linspace(0,-0.68,num)
# 	val3 = np.linspace(0,0.15,num)

# 	for i in range(num):
# 		v1 = val1[i]
# 		v2 = val2[i]
# 		v3 = val3[i]
# 		h_state.position[12:15] = [hval1,hval2,hval3,hval4]
# 		h_state.position[0:5] = [v1,v2,v3,val4,val5,-val5]
# 		state.position = [v1,v2,v3,val4,val5,-val5]
# 		pub.publish(state)
# 		pub_human.publish(h_state)

# 	#Second movement: Gripper on
# 	val5 = 0.1
# 	h_state.position[12:15] = [hval1,hval2,hval3,hval4]
# 	h_state.position[0:5] = [v1,v2,v3,val4,val5,-val5]
# 	state.position = [v1,v2,v3,val4,val5,-val5]
# 	pub.publish(state)
# 	pub_human.publish(h_state)

# 	#Third movement: Inwards
# 	val1 = np.linspace(v1,1.00,num)
# 	val2 = np.linspace(v2,0.22,num)
# 	val3 = np.linspace(v3,0.0,num)

# 	for i in range(num):
# 		v1 = val1[i]
# 		v2 = val2[i]
# 		v3 = val3[i]
# 		h_state.position[12:15] = [hval1,hval2,hval3,hval4]
# 		h_state.position[0:5] = [v1,v2,v3,val4,val5,-val5]
# 		state.position = [v1,v2,v3,val4,val5,-val5]
# 		pub.publish(state)
# 		pub_human.publish(h_state)

# 	#Fourth movement: grip open 
# 	val5 = 0.8
# 	h_state.position[12:15] = [hval1,hval2,hval3,hval4]
# 	h_state.position[0:5] = [v1,v2,v3,val4,val5,-val5]
# 	state.position = [v1,v2,v3,val4,val5,-val5]
# 	pub.publish(state)
# 	pub_human.publish(h_state)	

if __name__ == '__main__':
	rospy.init_node('publish_states_example_traj')
	
	third = ThirdArm()

	third.pubsub()

	# pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)
	# pub_human = rospy.Publisher('/joint_states', JointState, queue_size=1)

	# rospy.Subscriber('/joint_states', JointState, callback)

	#rate = rospy.Rate(5)
	#while not rospy.is_shutdown():


	
	
	
	#rospy.Subscriber('/joint_states', JointState, callback)
		
	# rospy.spin()
	#rate.sleep()
	