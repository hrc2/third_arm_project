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
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64, Int32, String, Float32MultiArray

import random
import arbotix_msgs.srv
import dynamixel_msgs.msg
import dynamixel_controllers.srv
from diagnostic_msgs.msg import DiagnosticArray
from apriltags_ros.msg import AprilTagDetectionArray
import csv
from pynput import keyboard
from prediction import thirdarm_logit

# Handover state: DoF 1 =  ?, DoF3 = Full_in
# Dropoff state: DoF 1 =  ?, DoF3 = Full_out

class hrc2d_speech_closed_loop:
    def __init__(self):
        rospy.init_node('reg_hrc2d_logit_motor_controller')

        self.currval = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command', Float64, queue_size=1)
        self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command', Float64, queue_size=1)
        self.pub_motor4 = rospy.Publisher('/wrist_controller/command', Float64, queue_size=1)
        self.pub_motor5 = rospy.Publisher('/wrist_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor6 = rospy.Publisher('/gripper_controller/command', Float64, queue_size=1)
        self.pubvec = [self.pub_motor1, self.pub_motor2, self.pub_motor3, self.pub_motor4, self.pub_motor5,
                       self.pub_motor6]

        self.pub_speech_command = rospy.Publisher('/arm_command_state', String, queue_size=1)
        self.pub_trial_number = rospy.Publisher('/hrc2d_task_number', Float64, queue_size=1)
        self.pub_target_probs = rospy.Publisher('/target_probs', Float32MultiArray, queue_size=1)

        self.trial_number = 0.0
        self.speed_topics = ['/base_swivel_controller/set_speed', '/vertical_tilt_controller/set_speed',
                             '/arm_extension_controller/set_speed', '/wrist_controller/set_speed',
                             '/wrist_tilt_controller/set_speed', '/gripper_controller/set_speed']
        self.motor_max_speeds = [0.5, 0.5, 1.6, 0.5, 0.5, 1.0]


        print('Setting motor max speeds')
        for i in range(len(self.motor_max_speeds)):
            self.set_motor_speeds(self.speed_topics[i], self.motor_max_speeds[i])

        #Assume extension starts from full out
        self.get_initial_states()

        self.grip_open = 0.0
        self.grip_close = 1.2

        self.d1_handover = 0.4
        self.d1_dropoff = -0.8

        self.full_out = self.currval[2]
        if self.full_out < 0.2:
            print('Error: Re-calibrate length extension')
            exit(0)
        #self.full_out = self.full_in - 2.0
        self.full_in = self.full_out + 1.6
        self.len_mid = 0.5*(self.full_out + self.full_in)
        if self.len_mid < self.full_out and self.len_mid > self.full_in:
            print('Error: Re-calibrate length extension')
            exit(0)

        #self.initial_angles = [0.0, -0.8, 0.5*(self.full_in + self.full_out), 0.0, -0.64, 0.0]
        self.initial_angles = [0.0, 0.0, self.len_mid, 0.0, -0.2, 0.0]

        print('Setting motor initial states')
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(self.initial_angles[i])
            time.sleep(1)

        self.calibrate()

        self.data_log_flag = 0
        self.train_flag = 0

        self.Xkt = np.array([])
        self.Xnt = np.array([])
        self.Ykt = np.array([])
        self.Ynt = np.array([])
        self.Tnt = np.array([])
        self.target1_positions = np.array([])
        self.target2_positions = np.array([])
        self.mean_target1 = np.array([])
        self.mean_target2 = np.array([])
        self.target_probs = [0.0, 0.0]
        self.set_target = 0
        # Later replace with  a structure depending on target clustering
        self.logit = thirdarm_logit()


    def set_motor_speeds(self, speed_topic, speed):
        self.set_speed = rospy.ServiceProxy(speed_topic, dynamixel_controllers.srv.SetSpeed)
        self.set_speed(speed)

    def get_initial_states(self):
        topic_list = ['/base_swivel_controller/state', '/vertical_tilt_controller/state',
                      '/arm_extension_controller/state', '/wrist_controller/state', '/wrist_tilt_controller/state',
                      '/gripper_controller/state']
        print("Waiting for joint states")
        self.count = 0
        for topic in topic_list:
            # print("Topic name : " + topic)
            data = rospy.wait_for_message(topic, dynamixel_msgs.msg.JointState)
            self.currval[self.count] = data.current_pos
            print("Topic name : " + topic + " Angle : " + str(data.current_pos))
            self.count += 1

        base_pose = rospy.wait_for_message('/base_pose', Point)
        ee_pose = rospy.wait_for_message('/ee_pose', Point)
        lh_pose = rospy.wait_for_message('/left_hand_pose', Point)
        rh_pose = rospy.wait_for_message('/right_hand_pose', Point)

        print('Detected Tags')
        self.theta = self.currval[0]
        self.l = self.currval[2]
        self.base_x = base_pose.x
        self.base_y = base_pose.y
        self.ee_x = ee_pose.x
        self.ee_y = ee_pose.y
        self.lh_x = lh_pose.x
        self.lh_y = lh_pose.y
        self.rh_x = rh_pose.x
        self.rh_y = rh_pose.y

    def calibrate(self):
        print('Calibrating length: 2 seconds')
        ee_pose = rospy.wait_for_message('/ee_pose', Point)
        self.ee_x = ee_pose.x
        self.ee_y = ee_pose.y
        self.thet0 = math.atan2((self.ee_y - self.base_y), (self.ee_x - self.base_x))
        self.l_mid = math.sqrt((self.base_x - self.ee_x) ** 2 + (self.base_y - self.ee_y) ** 2)
        #time.sleep(2)
        self.pubvec[2].publish(self.full_out)
        time.sleep(2)
        ee_pose = rospy.wait_for_message('/ee_pose', Point)
        self.ee_x = ee_pose.x
        self.ee_y = ee_pose.y
        self.l_max = math.sqrt((self.base_x - self.ee_x) ** 2 + (self.base_y - self.ee_y) ** 2)

        print('Calibration Done')

    def update_lh_pose(self, data):
        self.lh_x = data.x
        self.lh_y = data.y

    def update_rh_pose(self, data):
        self.rh_x = data.x
        self.rh_y = data.y

    def update_theta_state(self, data):
        self.theta = data.current_pos

    def update_l_state(self, data):
        self.l = data.current_pos

    def update_base_pose(self, data):
        self.base_x = data.x
        self.base_y = data.y
        self.data_prepare()

    def update_cup1_pose(self, data):
        self.cup1_x = data.x
        self.cup1_y = data.y

    def update_box1_pose(self, data):
        self.box1_x = data.x
        self.box1_y = data.y

    def update_ee_pose(self, data):
        self.ee_x = data.x
        self.ee_y = data.y
        self.pubvec[4].publish(-0.2)
        if self.set_target == 1 or self.set_target == 2:
            if self.data_log_flag == 1:
                self.move_to_target()

    def move_to_target(self):
        if self.set_target == 1:
            ee_target_x = self.mean_target1[0]
            ee_target_y = self.mean_target1[1]
        elif self.set_target == 2:
            ee_target_x = self.mean_target2[0]
            ee_target_y = self.mean_target2[1]
        else:
            return

        self.delta_l = 0.00
        self.theta_command = math.atan2((ee_target_y - self.base_y), (ee_target_x - self.base_x))
        self.l_command = math.sqrt((ee_target_x - self.base_x) ** 2 + (ee_target_y - self.base_y) ** 2) - self.delta_l

        m1_command = self.theta_command - self.thet0
        m3_command = self.full_out + (self.l_command - self.l_max) * \
                                     ((self.len_mid - self.full_out) / (self.l_mid - self.l_max))

        #print('Commands DoF1 : ' + str(m1_command) + ' DoF3 : ' + str(m3_command))

        if math.sqrt((ee_target_x - self.ee_x) ** 2 + (ee_target_y - self.ee_y) ** 2) <= 0.09:
            print('Found target. Opening gripper')
            self.pubvec[5].publish(self.grip_open)
            self.data_log_flag = 0
            self.set_target = 0
            self.pub_speech_command.publish('open hand')
            time.sleep(0.1)
            return

        if (m1_command > -1.57) and (m1_command < 1.57):
            #print('Moving DoF1')
            self.pubvec[0].publish(m1_command)
        if (m3_command > self.full_out) and (m3_command < self.full_in):
            #print('Moving DoF3')
            self.pubvec[2].publish(m3_command)

    def data_prepare(self):
        tk = time.time()
        if self.data_log_flag == 1:
            #print('Updating current data')
            X_current = [self.base_x, self.base_y, self.lh_x, self.lh_y, self.rh_x, self.rh_y]
            Y_current = [2 - (self.trial_number % 2)] # Update with target clustering part later on
            if(self.Xkt.size == 0):
                self.Xkt = np.array(X_current, ndmin=2)
                self.Ykt = np.array(Y_current, ndmin=2)
                self.Tnt = np.array(tk, ndmin=2)
            else:
                self.Xkt = np.append(self.Xkt, np.array(X_current, ndmin=2), axis=0)
                self.Ykt = np.append(self.Ykt, np.array(Y_current, ndmin=2), axis=0)
                self.Tnt = np.append(self.Tnt, np.array(tk, ndmin=2), axis=0)
            if self.trial_number > 2: #and self.set_target == 0:
                self.target_probs = self.logit.predict_probabilites(np.array(X_current, ndmin=2))
                probs = self.target_probs.ravel().tolist()
                self.pub_target_probs.publish(Float32MultiArray(data=probs))
                if probs[0] > 0.8:
                    self.set_target = 1
                elif probs[1] > 0.8:
                    self.set_target = 2

        elif self.data_log_flag == 0 and self.trial_number >= 1 and self.train_flag == 1:

            if(self.Xnt.size==0):
                self.Xnt = self.Xkt
                self.Ynt = self.Ykt
            else:
                self.Xnt = np.append(self.Xnt, self.Xkt, axis=0)
                self.Ynt = np.append(self.Ynt, self.Ykt, axis=0)
            self.Tnt = np.append(self.Tnt, np.array(tk, ndmin=2), axis=0)

            if self.trial_number % 2 == 1:
                if (self.target1_positions.size == 0):
                    self.target1_positions = np.array([self.ee_x, self.ee_y], ndmin=2)
                else:
                    self.target1_positions = np.append(self.target1_positions, np.array([self.ee_x, self.ee_y], ndmin=2),
                                                       axis=0)
                self.mean_target1 = np.mean(self.target1_positions, axis=0)

            elif self.trial_number % 2 == 0:
                if (self.target2_positions.size == 0):
                    self.target2_positions = np.array([self.ee_x, self.ee_y], ndmin=2)
                else:
                    self.target2_positions = np.append(self.target2_positions,
                                                       np.array([self.ee_x, self.ee_y], ndmin=2),
                                                       axis=0)
                self.mean_target2 = np.mean(self.target2_positions, axis=0)

            if self.trial_number >= 2:
                print('Training Logit model')
                self.logit.assign_data(self.Xnt, self.Ynt)
                self.logit.train()
            self.train_flag = 0



    def update_speech_input(self, dat):
        data = dat.data
        if data == 'close':
            print('Gripper Closing')
            self.pubvec[5].publish(self.grip_close)
            time.sleep(0.1)
        elif data == 'open':
            print('Gripper Opening')
            self.pubvec[5].publish(self.grip_open)
            self.data_log_flag = 0
            self.set_target = 0
            self.train_flag = 1
            time.sleep(0.1)
        elif data == 'go':
            self.pubvec[5].publish(self.grip_open)
            self.go_to_handover_location()
            #self.go_to_cup(self.cup1_x, self.cup1_y)
        elif data == 'put':
            self.trial_number += 1
            self.data_log_flag = 1
            self.pub_trial_number.publish(self.trial_number)
            self.go_to_dropoff()
        elif data == 'reset':
            self.go_to_init()
        elif data == 'stop':
            self.stop_motors()
        self.pub_speech_command.publish(data)

    def stop_motors(self):
        topic_list = ['/base_swivel_controller/state', '/vertical_tilt_controller/state',
                      '/arm_extension_controller/state', '/wrist_controller/state', '/wrist_tilt_controller/state',
                      '/gripper_controller/state']
        #print("Waiting for joint states")
        count = 0
        motor_angs = [0, 0, 0, 0, 0, 0]
        for topic in topic_list:
            # print("Topic name : " + topic)
            data = rospy.wait_for_message(topic, dynamixel_msgs.msg.JointState)
            motor_angs[count] = data.current_pos
            #print("Topic name : " + topic + " Angle : " + str(data.current_pos))
            count += 1
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(motor_angs[i])

    def go_to_handover_location(self):
        self.pubvec[2].publish(self.full_in)
        self.pubvec[0].publish(self.d1_handover)

    def go_to_dropoff(self):
        self.pubvec[2].publish(self.full_out)
        self.pubvec[0].publish(self.d1_dropoff)


    def go_to_cup(self, cup_x, cup_y):
        self.delta_l = 0.08
        self.theta_command = math.atan2((cup_y - self.base_y), (cup_x - self.base_x))
        self.l_command = math.sqrt((cup_x - self.base_x) ** 2 + (cup_y - self.base_y) ** 2) - self.delta_l

        m1_command = self.theta_command - self.thet0
        m3_command = self.full_out + (self.l_command - self.l_max) * \
                                     ((self.len_mid - self.full_out) / (self.l_mid - self.l_max))

        print('Commands DoF1 : ' + str(m1_command) + ' DoF3 : ' + str(m3_command))
        print('DoF 3 params. [l_cmd, l_max, l_mid, f_out, f_in, len_mid] '
              + str([self.l_command, self.l_max, self.l_mid, self.full_out, self.full_in, self.len_mid]))

        if (m1_command > -1.57) and (m1_command < 1.57):
            print('Moving DoF1')
            self.pubvec[0].publish(m1_command)
        if (m3_command > self.full_out) and (m3_command < self.full_in):
            print('Moving DoF3')
            self.pubvec[2].publish(m3_command)

    def cup_to_box(self, box_x, box_y):
        self.delta_bx = 0.00
        self.delta_by = 0.00
        self.theta_command = math.atan2((box_y - self.base_y - self.delta_by), (box_x - self.base_x - self.delta_bx))
        self.l_command = math.sqrt((box_x - self.base_x - self.delta_bx) ** 2 + (box_y - self.base_y - self.delta_by) ** 2)

        m1_command = self.theta_command - self.thet0
        m3_command = self.full_out + (self.l_command - self.l_max) * \
                                     ((self.len_mid - self.full_out) / (self.l_mid - self.l_max))

        print('Commands DoF1 : ' + str(m1_command) + ' DoF3 : ' + str(m3_command))
        print('DoF 3 params. [l_cmd, l_max, l_mid, f_out, f_in, len_mid] '
              + str([self.l_command, self.l_max, self.l_mid, self.full_out, self.full_in, self.len_mid]))

        if (m1_command > -1.57) and (m1_command < 1.57):
            print('Moving DoF1')
            self.pubvec[0].publish(m1_command)
        if (m3_command > self.full_out) and (m3_command < self.full_in):
            print('Moving DoF3')
            self.pubvec[2].publish(m3_command)

    def go_to_init(self):
        print('Setting motors to initial states')
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(self.initial_angles[i])
            time.sleep(0.5)
        self.pubvec[2].publish(self.full_out)

    def run(self):
        rospy.Subscriber('/base_swivel_controller/state', dynamixel_msgs.msg.JointState, self.update_theta_state)
        rospy.Subscriber('/arm_extension_controller/state', dynamixel_msgs.msg.JointState, self.update_l_state)
        rospy.Subscriber('/base_pose', Point, self.update_base_pose)
        rospy.Subscriber('/ee_pose', Point, self.update_ee_pose)
        rospy.Subscriber('/left_hand_pose', Point, self.update_lh_pose)
        rospy.Subscriber('/right_hand_pose', Point, self.update_rh_pose)
        rospy.Subscriber('/recognizer/output', String, self.update_speech_input)
        rospy.spin()

if __name__ == '__main__':
    t1 = hrc2d_speech_closed_loop()
    t1.run()
