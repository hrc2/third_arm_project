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

# Test out the tuned PID controllers for ech dof

class ThirdArm:	

	h_names = ['base_to_motor1', 'swivel_to_motor2', 'extension', 'wrist_motor_to_extension', 'finger1_to_gripper_motor', 'finger2_to_gripper_motor', 'spine_0', 'spine_1', 'spine_2', 'neck_0', 'neck_1', 'neck_2', 'right_shoulder_0', 'right_shoulder_1', 'right_shoulder_2', 'right_elbow_0', 'right_wrist_0', 'right_wrist_1', 'right_wrist_2', 'left_shoulder_0', 'left_shoulder_1', 'left_shoulder_2', 'left_elbow_0', 'left_wrist_0', 'left_wrist_1', 'left_wrist_2', 'right_hip_0', 'right_hip_1', 'right_hip_2', 'right_knee_0', 'right_ankle_0', 'right_ankle_1', 'left_hip_0', 'left_hip_1', 'left_hip_2', 'left_knee_0', 'left_ankle_0', 'left_ankle_1']
	arm_names = ['base_to_motor1', 'swivel_to_motor2', 'extension', 'wrist_motor_to_extension', 'finger1_to_gripper_motor', 'finger2_to_gripper_motor']
	dofvals = []
	hvals = []
	torque = [0.0 for i in range(5)]
	h_state = JointState([],h_names,hvals,[],[])
	h_len = len(h_names)
	state = JointState([],h_names,dofvals,[],[])

	flag = 0
	end = 10
	goal_data = []
	pos_data = []

	def push_data(self):	
		
		self.set_data1()
		self.send_motor()
		self.flag = self.flag + 1

	def send_motor(self):
	# base_val = -self.h_state.position[0]
		# tilt_val = -0.3 + (-self.h_state.position[1])
		# extend_val = -0.3 + (-2.0+0.3)*self.h_state.position[2]/0.15
		# gripper_val =  1.3 - self.h_state.position[4]
		base_val = -2.9 # -3.1 to -2.4, -2.9init
		tilt_val = -2.9 #-2.4 to -3.1, -2.9init
		extend_val = -1.0 #-1.0 to -2.8, -1.0init
		gripper_val =  self.dofvals[4] #0.6 to 1.6, 1.60init 
		wrist_val = 0.0 #-1.57/3 to 1.57/3, 0init
		self.pub_motor5.publish(gripper_val)
		self.pub_motor3.publish(extend_val)
		self.pub_motor2.publish(tilt_val)		
		self.pub_motor1.publish(base_val)
		self.pub_motor4.publish(wrist_val)

	def print_position(self,data):
		
		self.position = data.motor_states[0].position
		
		self.pos_data.append(self.position)
		# with open('sys_id_dof3_pos.csv','a') as f:
		#  	writer = csv.writer(f,quoting=csv.QUOTE_ALL)
		#  	writer.writerow(write_data)

		self.goal = data.motor_states[0].goal

		self.goal_data.append(self.goal)
		# with open('sys_id_dof3_goal.csv','a') as f2:
		#  	writer = csv.writer(f2,quoting=csv.QUOTE_ALL)
		#  	writer.writerow(write_data2)


		#rospy.loginfo('\nPosition is: {0} \n'.format(write_data))	
		
	def set_data1(self):
		if self.flag%2 == 0:
			base_val = -2.4
			tilt_val = -2.4
			extend_val = -2.8
			wrist_val = -1.57/3.0
			gripper_val = 0.6
		else: 
			base_val = -3.1
			tilt_val = -3.1
			extend_val = -1.0
			wrist_val = 1.57/3.0
			gripper_val = 1.63
		
		self.dofvals = [base_val,tilt_val,extend_val,wrist_val,gripper_val];

	def sub_once(self,data):
		self.state = data
		self.h_len = len(data.name)
		self.h_state = data
		rospy.loginfo('I subbed:{0}'.format(self.h_len))

		self.sub.unregister()

	def pubsub(self):
		self.rate = rospy.Rate(0.5)

		self.pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)
		self.pub_human = rospy.Publisher('/joint_states', JointState, queue_size=1)
		self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command',Float64, queue_size=1)
		self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command',Float64, queue_size=1)
		self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command',Float64, queue_size=1)
		self.pub_motor4 = rospy.Publisher('/wrist_controller/command',Float64, queue_size=1)
		self.pub_motor5 = rospy.Publisher('/gripper_controller/command',Float64, queue_size=1)

		while self.flag < self.end:
			self.sub = rospy.Subscriber('/joint_states', JointState, self.sub_once)
			#self.motor_sub = rospy.Subscriber('/gripper_controller/state', dynamixel_msgs.msg.JointState, self.get_torque)
			
			self.push_data()
			self.rate.sleep()
			self.motor_sub = rospy.Subscriber('/motor_states/third_arm_port', dynamixel_msgs.msg.MotorStateList, self.print_position)

		with open('pid_tune_dof5_pos.csv','a') as f:
		 	writer = csv.writer(f,quoting=csv.QUOTE_ALL)
		 	writer.writerow(self.pos_data)

		with open('pid_tune_dof5_goal.csv','a') as f2:
		 	writer = csv.writer(f2,quoting=csv.QUOTE_ALL)
		 	writer.writerow(self.goal_data)
		
		#rospy.spin()
			





if __name__ == '__main__':
	rospy.init_node('pid_tune_motor_test')
	
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
	