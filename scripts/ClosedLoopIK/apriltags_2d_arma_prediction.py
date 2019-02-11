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

from statsmodels.tsa.arima_model import ARMA
from statsmodels.tsa.ar_model import AR

CHOICE = 0 # 0 for AR model, 1 for ARMA model
ARFILE = 'data/ar_params.csv'
ARMAFILE = 'data/arma_params.csv'

class apriltags_2d_predict:
	
	def __init__(self):
        rospy.init_node('apriltags_2d_predict')
        self.pub_base_predict_next = rospy.Publisher('/base_pose_next_prediction', Point, queue_size=1) # Predict next time step
        self.pub_base_predict_nth = rospy.Publisher('/base_pose_nth_prediction', Point, queue_size=1) # Predict N time steps into the future
        self.N = 10
        self.base_pos = rospy.wait_for_message('/base_pose', Point)
        print("Base Position Message Detected")
        self.msg_count = 0
        self.param_read()

    def param_read(self):
    	if CHOICE == 0:
    		self.pred_params = np.genfromtxt(ARFILE, delimiter=',')
    		self.pos_buffer = np.zeros([self.pred_params.shape[0], 3])
        elif CHOICE == 1:
        	self.pred_params = np.genfromtxt(ARMAFILE, delimiter=',')

    def update_buffer(self, data):
    	if self.msg_count < self.pred_params.shape[0]:
    		self.pos_buffer[self.msg_count, :] = np.array([data.x, data.y, data.z])
    		self.msg_count++
    	else:
    		self.pos_buffer = np.roll(self.pos_buffer, 1, axis=0)
    		self.pos_buffer[-1, :] = np.array([data.x, data.y, data.z])
    		self.next_predict()
    		

    def next_predict(self):
    	print("Predicting one time step into the future")
    	self.next_prediciton = np.matmul(self.pos_buffer, np.flip(self.pred_params, 0))
    	self.pub_base_predict_next.publish(self.next_prediciton)
    
    def run(self):
    	self.base_pos_sub = rospy.Subscriber('/base_pose', Point, self.update_buffer)
		rospy.spin()


if __name__ == '__main__':
    t1 = apriltags_2d_predict()
    t1.run()
    	