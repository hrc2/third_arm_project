#!/usr/bin/env python
import rospy

import math
import time
import numpy as np
import sys
import tf
import PyKDL
import dynamic_reconfigure.server
import argparse

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
from apriltags_ros.msg import AprilTagDetectionArray
import csv

class apriltags_motor_control: 


    def __init__(self):
        rospy.init_node('apriltags_motor_controller')
	
	self.currval = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    	self.command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]	

        self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command', Float64, queue_size=1)
        self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command', Float64, queue_size=1)
        self.pub_motor4 = rospy.Publisher('/wrist_controller/command', Float64, queue_size=1)
        self.pub_motor5 = rospy.Publisher('/wrist_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor6 = rospy.Publisher('/gripper_controller/command', Float64, queue_size=1)
	
	self.pubvec = [self.pub_motor1, self.pub_motor2, self.pub_motor3, self.pub_motor4, self.pub_motor5, self.pub_motor6]
	self.initial_angles = [0.0, -0.06, 1.5, 0.0, -0.64, 0.0]	

        self.speed_topics = ['base_swivel_controller/set_speed', '/vertical_tilt_controller/set_speed' , '/arm_extension_controller/set_speed' , '/wrist_controller/set_speed' , '/wrist_tilt_controller/set_speed', '/gripper_controller/set_speed']
        self.motor_max_speeds = [0.4, 0.2, 0.8, 0.5, 0.5, 0.5]
	
	print('Setting motor max speeds')
	for i in range(len(self.motor_max_speeds)):
		self.set_motor_speeds(self.speed_topics[i], self.motor_max_speeds[i])
	print('Setting motor initial states')	
	for i in range(len(self.initial_angles)):
		self.pubvec[i].publish(self.initial_angles[i])
		time.sleep(0.5)

    def set_motor_speeds(self, speed_topic, speed):
    	self.set_speed = rospy.ServiceProxy(speed_topic, dynamixel_controllers.srv.SetSpeed)
    	self.set_speed(speed)

    def get_initial_motor_states(self):
        topic_list = ['/base_swivel_controller/state' , '/vertical_tilt_controller/state' , '/arm_extension_controller/state' , '/wrist_controller/state' , '/wrist_tilt_controller/state', '/gripper_controller/state']
        print("Waiting for joint states")
        self.count = 0
        for topic in topic_list:
            #print("Topic name : " + topic)
            data = rospy.wait_for_message(topic, dynamixel_msgs.msg.JointState)            
            self.currval[self.count] = data.current_pos
            print("Topic name : " + topic + " Angle : " + str(data.current_pos))
            self.count += 1

    def get_initial_frame_poses(self):	
        t0 = time.time()
        t = time.time()
        timeout = 2.0
	print("Setting initial pose values: " + str(timeout) +" seconds averaged")
        self.base_x_avg = 0.0
        self.base_y_avg = 0.0
        self.ee_x_avg = 0.0
        self.ee_y_avg = 0.0
        count = 0
        while(t < t0 + timeout):
		base_pose = rospy.wait_for_message('/base_pose', Point)
		ee_pose = rospy.wait_for_message('/ee_pose', Point)
		self.base_x = base_pose.x           
		self.base_y = base_pose.y
		self.ee_x = ee_pose.x
		self.ee_y = ee_pose.y
		self.base_x_avg += base_pose.x
		self.base_y_avg += base_pose.y
		self.ee_x_avg += ee_pose.x
		self.ee_y_avg += ee_pose.y    	   
		count += 1
		t = time.time()

        self.base_x_avg /= count
        self.base_y_avg /= count
        self.ee_x_avg /= count
        self.ee_y_avg /= count
        print("Base- X: " + str(self.base_x_avg) + " Y: " + str(self.base_y_avg))
        print("Gripper- X: " + str(self.ee_x_avg) + " Y: " + str(self.ee_y_avg))
	
    def update_base_pose(self, data):
	self.base_x = data.x
	self.base_y = data.y
        print("Base- X: " + str(self.base_x) + " Y: " + str(self.base_y))
    
    def update_ee_pose(self, data):
	self.ee_x = data.x
	self.ee_y = data.y
        print("Gripper- X: " + str(self.ee_x) + " Y: " + str(self.ee_y))

    def closed_loop_control(self):
	time.sleep(0.5)
	print('Closed Loop Controller')
	
    def run(self):
	#rospy.Subscriber("joy", Joy, self.test_joystick)
	#rospy.Subscriber("joy", Joy, self.get_joystick)
	#rospy.spin()
	self.get_initial_motor_states()
    	self.get_initial_frame_poses()
	print("Subscribing to topics:")
	rospy.Subscriber('/base_pose', Point, self.update_base_pose)
	rospy.Subscriber('/ee_pose', Point, self.update_ee_pose)
	self.closed_loop_control()	
	rospy.spin()
		#self.get_initial_motor_states()
		#self.get_frame_poses()
		
if __name__ == '__main__':
    t1 = apriltags_motor_control()
    t1.run()
