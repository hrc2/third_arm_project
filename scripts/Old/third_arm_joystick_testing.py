#!/usr/bin/env python
# Arm testing script to find the cause of delay in motion from joystick

import rospy

import math
import numpy as np
import sys
import tf
import PyKDL
import dynamic_reconfigure.server
import argparse
import time

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
import csv

class thirdarm_js_test:
	currval = [0.0, 0.0, 0.0, 0.0, 0.0]
	command = [0.0, 0.0, 0.0, 0.0, 0.0]

	def __init__(self):
		rospy.init_node('thirdarm_joystick_testing')

		self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command', Float64, queue_size=1)      
		self.set_speed1 = rospy.ServiceProxy('base_swivel_controller/set_speed', dynamixel_controllers.srv.SetSpeed)

		self.js = Joy

	def check(self,ax,bu):
		ch = 0
		if np.count_nonzero(np.asarray(ax)) >= 1:
			ch = 1
		elif np.count_nonzero(np.asarray(bu)) >= 1:
			ch = 1
		else:
			ch = 0

		#print ("Check: " + str(ch))
		return ch

	def run(self):
		t_prev = time.time()
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.js = rospy.wait_for_message("joy", Joy)
			t_new = time.time()
			rospy.loginfo('Rate = {0}'.format(1.0/(t_new-t_prev)))
			t_prev = t_new			
			if self.check(self.js.axes , self.js.buttons) == 1: #Non-zero input received
				self.get_send_joystick()
			else:
				continue			
			r.sleep()
			

	def get_current_state(self):
		topic = '/base_swivel_controller/state'
		print("Waiting for joint states")
		self.count = 0        		
		#print("Topic name : " + topic)
		data = rospy.wait_for_message(topic, dynamixel_msgs.msg.JointState)            
		self.currval[self.count] = data.current_pos
		#self.count += 1
		#print("JS data: " + str(self.currval))   	

	def get_send_joystick(self):
		self.get_current_state()

		js = self.js

		js_cmd = js.axes[4]
		js_fine = js.buttons[7]

		if js_fine == 1:
			base_cmd = -0.15
		else:
			base_cmd = -0.3

		cmd_scale = [base_cmd, 0.3, -0.4, -0.3, 0.2]	

		command = self.currval[0] + np.sign(js_cmd)*cmd_scale[0]
		print('Command = '+str(command))
		self.set_speed1((math.pi/0.8)*np.fabs(cmd_scale[0]))
		self.pub_motor1.publish(command)

		# rate = 50 #Rate in Hertz
		# r = rospy.Rate(rate)
		# x = 0.0
		# x_step = cmd_scale[0]/10.0
		# #self.pub_motor1.publish(0.0)
		# #time.sleep(1)
		# while x < cmd_scale[0]:#2.0*math.pi: #rospy.is_shutdown(): #
		# 	#print('Something!')
		# 	command = self.currval[0] + np.sign(js_cmd)*x
		# 	#command = 0.9*math.pi*math.sin(x)
		# 	#print('Computed!')
		# 	print('x: '+str(x)+' cmd: '+str(command))
		# 	self.pub_motor1.publish(command)
		# 	#pub_motor2.publish(0.0)
		# 	x += x_step
		# 	r.sleep()

    	#print("Command Values is " + str(self.command))        
        




if __name__ == '__main__':
	t1 = thirdarm_js_test()
	t1.run()




# import pygame
# from pygame.locals import *
# from sys import exit
# from bluepy import btle
# import math
# import struct
# #import BB8_driver
# #from sphero_driver import sphero_driver
# #from sphero import SpheroNode

# import rospy

# import math
# import numpy as np
# import sys
# import tf
# import PyKDL
# import dynamic_reconfigure.server
# import argparse

# import sensor_msgs.msg
# from sensor_msgs.msg import Imu, JointState, Joy
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
# from std_msgs.msg import ColorRGBA, Float32, Bool, Float64

# #import rospy
# #import roslib
# import random
# #import math
# import arbotix_msgs.srv
# #from std_msgs.msg import String, Float64
# #from geometry_msgs.msg import Vector3, Twist
# #from sensor_msgs.msg import JointState
# import dynamixel_msgs.msg
# from diagnostic_msgs.msg import DiagnosticArray
# #import numpy as np
# import csv


# class thirdarm_joystick:    

#     currval = [0.0, 0.0, 0.0, 0.0, 0.0]
#     command = [0.0, 0.0, 0.0, 0.0, 0.0]

    

#     def __init__(self):
#         rospy.init_node('thirdarm_joystick')
        
#         pygame.init()
#         pygame.joystick.init()
#         self.j1 = pygame.joystick.Joystick(0)
#         self.j1.init()
#         self.count = 0

#         #print pygame.joystick.get_count()
        

#         self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command',Float64, queue_size=1)
#         self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command',Float64, queue_size=1)
#         self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command',Float64, queue_size=1)
#         self.pub_motor4 = rospy.Publisher('/wrist_controller/command',Float64, queue_size=1)
#         self.pub_motor5 = rospy.Publisher('/gripper_controller/command',Float64, queue_size=1)

#         self.pubvec = [self.pub_motor1,self.pub_motor2,self.pub_motor3,self.pub_motor4,self.pub_motor5 ]
    
#     def arm_callback(self, data):
#         print(data)
#         self.currval[self.count] = data.position
#         #for i in range(len(self.currval)):
#          #   self.currval[i] = data.motor_states[i].position
#             #self.currval[i] = float(data.status[i].values[1].value)
#             #print self.currval[i]
#         #print("Current DoF values are: " + str(self.currval))
#         #self.get_joystick() 
        
        

#     def get_joystick(self):
#         pygame.event.wait()
#         # Axis, buttons in use:
#         # Axis 0: D-Pad Left Right
#         # Axis 1: D-Pad Up Down
#         # Axis 2: Right Analog Left Right
#         # Axis 3: Right Analog Up Down
#         # Axis 4: Left Analog Left Right
#         # Axis 5: Left Analog Up Down
#         # Button 0: Blue X
#         # Button 1: Green A

#         # Mapping to DoFs:
#         # Dof1: Base Swivel: Axis 2 : Right analog L/R
#         # Dof2: Vertical pitching: Hat 0 : Left analog U/D
#         # Dof3: Length extension: Axis 1 : D-Pad U/D
#         # Dof4: Wrist rotation: Axis 0 : D-Pad L/R
#         # Gripper: Button 0 : Button X



#         #c1 = self.j1.get_axis(2)
#         self.cx1 = self.j1.get_button(6)
#         self.cx2 = self.j1.get_button(7)
#         (cx,c2) = self.j1.get_hat(0)
#         #c2 = c2*(-1)
#         c3 = self.j1.get_axis(1)
#         c4 = self.j1.get_axis(0)
#         c5 = self.j1.get_button(0)
#         c6 = self.j1.get_button(1)        

#         # Assign commands for each Dof:
#         c1  = 0
#         if self.cx1>0:
#             c1 = -1
#         elif self.cx2>0:
#             c1 = 1

#         self.cvec = [c1,c2,c3,c4,c5,c6]
#         # cmd_scale = [0.1, 0.1, 0.1, 0.1, 1]

#         # grip_close = -0.1
#         # grip_open = 0.1

#         # for i in range(len(cvec)):
#         #     if not i == 4:
#         #         self.command[i] = self.currval[i] + np.sign(cvec[i])*cmd_scale[i]
#         #     elif c5 > 0 :
#         #         self.command[i] = self.currval[i] + grip_close
#         #     elif c6 > 0 :
#         #         self.command[i] = self.currval[i] + grip_open
#             # elif cvec[i] == True:
#             #     if math.fabs(self.currval[i] - grip_close)<0.2:
#             #         self.command[i] = grip_open
#             #     else:
#             #         self.command[i] = grip_close
        
#         #Temp code        
        

#         #rospy.loginfo('Command to be published: {0}', str(self.command))                    
#         #print self.command
#         #self.test_seq()
#         self.publish_cmd()

#     def get_current_state(self):
#         topic_list = ['/base_swivel_controller/state' , '/vertical_tilt_controller/state' , '/arm_extension_controller/state' , '/wrist_controller/state' , '/gripper_controller/state']
#         print("Waiting for joint states")
#         self.count = 0
#         for topic in topic_list:
#             print("Topic name : " + topic)
#             #self.motor_sub = rospy.Subscriber(topic, dynamixel_msgs.msg.JointState, self.arm_callback)
#             #self.count +=  1           
#             data = rospy.wait_for_message(topic, dynamixel_msgs.msg.JointState)
#             print("JS data: " + str(data.current_pos))
#             self.currval[self.count] = data.current_pos
#             self.count += 1
#             # print data.position


#     def publish_cmd(self):

#         cmd_scale = [0.3, 0.5, 0.6, 0.5, 1]

#         grip_close = -0.3
#         grip_open = 0.3

#         #data = rospy.wait_for_message('/diagnostics', DiagnosticArray)
#         self.get_current_state()        
#         #rospy.Subscriber('/diagnostics', DiagnosticArray, self.arm_callback)           
#         # for i in range(len(self.currval)):
#         #     #self.currval[i] = data.motor_states[i].position
#         #     self.currval[i] = float(data.status[i].values[1].value)
        
#         print("Current DoF values are: " + str(self.currval))

#         for i in range(len(self.cvec) - 1):
#             if i < 4:
#                 self.command[i] = self.currval[i] + np.sign(self.cvec[i])*cmd_scale[i]
#             elif self.cvec[4] > 0 :
#                 self.command[i] = self.currval[i] + grip_close
#             elif self.cvec[5] > 0 :
#                 self.command[i] = self.currval[i] + grip_open
        
#         print("Command Values are " + str(self.command))
        
        
#         for i in range(len(self.pubvec)):
#            self.pubvec[i].publish(self.command[i])

#         #self.cmd_reset()
    
#     def cmd_reset(self):
#         self.command = [0.0, 0.0, 0.0, 0.0, 0.0]

#     def execute(self):
#         r = rospy.Rate(10.0)
#         clock = pygame.time.Clock()
#         while not rospy.is_shutdown():
#             clock.tick(10)
#             #self.motor_sub = rospy.Subscriber('/diagnostics', dynamixel_msgs.msg.MotorStateList, self.arm_callback)           
#             self.motor_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self.arm_callback)           
#             r.sleep()
    
#     def run(self):
#         #clock = pygame.time.Clock()
#         while not rospy.is_shutdown():
#             #pygame.time.delay(10)
#             self.get_joystick()

#     def test_joystick(self):
#         pygame.event.wait()
#         name = self.j1.get_name()
#         print "Joystick name: " + name + "\n"
        
#         axes = self.j1.get_numaxes()
#         print "Joystick axes: {}".format(axes) 
#         for i in range(axes):
#             ax = self.j1.get_axis(i)
#             print "Axis {0} Value {1}".format(i,ax)        

#         buttons = self.j1.get_numbuttons()
#         print "Joystick buttons: {}".format(buttons) 
#         for i in range(buttons):
#             ax = self.j1.get_button(i)
#             print "Button {0} Value {1}".format(i,ax)        
        
#         hats = self.j1.get_numhats()
#         print "Joystick hats: {}".format(hats) 
#         for i in range(hats):
#             ax = self.j1.get_hat(i)
#             print "Hat {0} Value {1}".format(i,ax)        


#     def test(self):
#         clock = pygame.time.Clock()
#         #r = rospy.Rate(10.0)
#         while not rospy.is_shutdown():
#             clock.tick(10)
#             self.get_joystick()
        


# if __name__ == '__main__':
#     t1 = thirdarm_joystick()
#     t1.run()
#     