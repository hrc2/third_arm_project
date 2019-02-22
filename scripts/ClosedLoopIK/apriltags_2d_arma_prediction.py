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
import os

import sensor_msgs.msg
from sensor_msgs.msg import Imu, JointState, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64, Int32

import random
import arbotix_msgs.srv
import dynamixel_msgs.msg
import dynamixel_controllers.srv
from diagnostic_msgs.msg import DiagnosticArray
from apriltags_ros.msg import AprilTagDetectionArray
import csv

from statsmodels.tsa.arima_model import ARMA
from statsmodels.tsa.ar_model import AR

CHOICE = 0  # 0 for AR model, 1 for ARMA model
ARFILE = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/ar_params_2d.csv'
AR_SIGMA_FILE = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/ar_sigmas.csv'
ARMAFILE = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/arma_params.csv'
GROUND_TRUTH = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/base_pos.csv'
EE_POSE = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/ee_pos.csv'
PREDICTION_ONE_STEP = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/base_predict_next.csv'
PREDICTION_NTH_STEP = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/base_predict_nth.csv'
WITHIN_RANGE_FLAG = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/within_range.csv'


class apriltags_2d_predict:
    def __init__(self):
        rospy.init_node('apriltags_2d_predict')
        print("Initializing")
        self.pub_base_predict_next = rospy.Publisher('/base_pose_next_prediction', Point,
                                                     queue_size=1)  # Predict next time step
        self.pub_base_predict_nth = rospy.Publisher('/base_pose_nth_prediction', Point,
                                                    queue_size=1)  # Predict N time steps into the future
        self.N = 0
        self.Nplus = 1
        self.msg_count = 0
        self.init_time = time.time()

    def param_read(self):
        print("Read AR/ARMA parameters from file")
        if CHOICE == 0:
            self.pred_params = np.genfromtxt(ARFILE, delimiter=',')
            self.pos_buffer = np.zeros([self.pred_params.shape[0], 3])
            self.sigma = np.sqrt(np.genfromtxt(AR_SIGMA_FILE, delimiter=''))
            self.N = self.pred_params.shape[0]
        elif CHOICE == 1:
            self.pred_params = np.genfromtxt(ARMAFILE, delimiter='')

    def update_buffer(self, data):
        if self.msg_count < 5*self.pred_params.shape[0]:
            self.pos_buffer = np.roll(self.pos_buffer, 1, axis=0)
            self.pos_buffer[-1, :] = np.array([data.x, data.y, data.z])
            self.write_to_file(GROUND_TRUTH, np.array([data.x, data.y, data.z]))
            self.msg_count += 1
        else:
            self.pos_buffer = np.roll(self.pos_buffer, 1, axis=0)
            self.pos_buffer[-1, :] = np.array([data.x, data.y, data.z])
            self.write_to_file(GROUND_TRUTH, np.array([data.x, data.y, data.z]))
            self.next_predict()
            #self.n_step_predict(N=1, fname=PREDICTION_ONE_STEP) # One step with lag compensation
            #self.n_step_predict(N=self.N + self.Nplus, fname=PREDICTION_NTH_STEP)
            self.n_step_predict(N=2, fname=PREDICTION_NTH_STEP)


    def next_predict(self):
        # print("Predicting one time step into the future")
        self.next_pred = np.zeros(self.pos_buffer.shape[1])
        for i in range(self.pos_buffer.shape[1]):
            self.next_pred[i] = np.dot(np.flip(self.pos_buffer[:, i], 0), self.pred_params[:, i]) #+ np.random.normal(0, self.sigma[i]**2)
        self.next_prediciton_msg = Point(x=self.next_pred[0], y=self.next_pred[1], z=self.next_pred[2])
        self.pub_base_predict_next.publish(self.next_prediciton_msg)
        self.write_to_file(PREDICTION_ONE_STEP, self.next_pred)

    def n_step_predict(self, N, fname):
        #print("Predicting " + str(self.N) + " time steps into the future")
        curr_buffer = self.pos_buffer
        self.nth_pred = np.zeros(self.pos_buffer.shape[1])
        for j in range(N):
            for i in range(self.pos_buffer.shape[1]):
                self.nth_pred[i] = np.dot(np.flip(curr_buffer[:, i], 0), self.pred_params[:, i]) #+ np.random.normal(0, self.sigma[i]**2)
            curr_buffer = np.roll(curr_buffer, 1, axis=0)
            curr_buffer[-1, :] = self.nth_pred
        self.nth_prediciton_msg = Point(x=self.nth_pred[0], y=self.nth_pred[1], z=self.nth_pred[2])
        #self.pub_base_predict_nth.publish(self.nth_prediciton_msg)
        nth_filt = np.zeros(3)
        nth_filt[0] = 0.95*self.next_pred[0] + 0.05*self.nth_pred[0]
        #nth_filt[1] = 0.999 * self.next_pred[1] + 0.0005 * self.nth_pred[1]
        nth_filt[1] = self.pos_buffer[-1, 1]
        nth_filt[2] = self.next_pred[2]
        self.pub_base_predict_nth.publish(Point(x=nth_filt[0], y=nth_filt[1], z=nth_filt[2]))
        self.write_to_file(fname, self.nth_pred)

    def write_to_file(self, fname, data):
        a = 10

        #with open(fname, 'a') as f:
        #    np.savetxt(f, [np.append(data, time.time() - self.init_time)], fmt='%1.10f', delimiter=',')

    def log_end_effector(self, data):
        self.write_to_file(EE_POSE, np.array([data.x, data.y, data.z]))

    def log_within_range(self, data):
        self.write_to_file(WITHIN_RANGE_FLAG, data)

    def run(self):
        self.base_pos = rospy.wait_for_message('/base_pose', Point)
        #self.ee_pos = rospy.wait_for_message('/base_pose', Point)
        print("Base Position Message Detected")
        self.param_read()
        self.base_pos_sub = rospy.Subscriber('/base_pose', Point, self.update_buffer)
        self.ee_pos_sub = rospy.Subscriber('/ee_pose', Point, self.log_end_effector)
        self.range_flag_sub = rospy.Subscriber('/within_range', Int32, self.log_within_range)
        rospy.spin()


if __name__ == '__main__':
    t1 = apriltags_2d_predict()
    t1.run()
