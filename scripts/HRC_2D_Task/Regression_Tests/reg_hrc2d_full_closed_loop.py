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
from datetime import date

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
from target_prediction import thirdarm_logit
from task_prediction import thirdarm_task_prediction
from sklearn.cluster import KMeans
from jumpsmethod import JumpsMethod
from sklearn.externals import joblib

# Handover state: DoF 1 =  ?, DoF3 = Full_in
# Dropoff state: DoF 1 =  ?, DoF3 = Full_out
path = os.path.dirname(__file__)
today = str(date.today()) + '-' + str(int(time.time()))
logit_save_file = str(path + '/Data/Trained_Models/Logit_Target_Model' + today + '.sav')
knn_save_file = str(path + '/Data/Trained_Models/KNN_Task_Model' + today + '.sav')
logit_load_file = str(path + '/Data/Trained_Models/Logit_Target_Model2019-07-19-1563521135')
knn_load_file = str(path + '/Data/Trained_Models/KNN_Task_Model2019-07-19-1563521135')

class hrc2d_closed_loop:
    def __init__(self):
        rospy.init_node('hrc2d_motor_controller')

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
        self.pub_task_probs = rospy.Publisher('/task_probs', Float32MultiArray, queue_size=1)

        self.trial_number = 1.0
        self.speed_topics = ['/base_swivel_controller/set_speed', '/vertical_tilt_controller/set_speed',
                             '/arm_extension_controller/set_speed', '/wrist_controller/set_speed',
                             '/wrist_tilt_controller/set_speed', '/gripper_controller/set_speed']
        self.motor_max_speeds = [0.4, 0.5, 1.0, 1.0, 1.0, 1.6]


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

        self.Xkt = np.array([])
        self.Xnt = np.array([])
        self.Ykt = np.array([])
        self.Ynt = np.array([])
        self.Tnt = np.array([])
        self.target_probs = [0.0, 0.0]
        self.set_target = 0

        self.target_logit = thirdarm_logit()
        self.num_targets = 0
        self.target_positions = np.array([])
        self.jump_init_number = 4
        self.target_labels = np.array([])
        self.target_means = np.array([])
        
        self.task_predict = thirdarm_task_prediction()
        self.relevant_commands = ['close', 'go', 'put', 'stop']
        self.dn_command = ['stop']
        self.task_predict.set_relevant_commands(self.relevant_commands, self.dn_command)
        self.task_pred_probs = [0.00, 0.0, 0.0, 0.0, 0.0]
        self.grip_open_prob = 0.0

        self.pred_state_prev = ''
        self.speech_current = ''
        self.mode = ''
        self.prev_cmd = ''

        self.pub_task_probs.publish(Float32MultiArray(data=self.task_pred_probs))
        self.pub_target_probs.publish(Float32MultiArray(data=self.target_probs))
        self.pub_trial_number.publish(self.trial_number)

        rospy.Subscriber('/base_swivel_controller/state', dynamixel_msgs.msg.JointState, self.update_theta_state)
        rospy.Subscriber('/arm_extension_controller/state', dynamixel_msgs.msg.JointState, self.update_l_state)
        rospy.Subscriber('/base_pose', Point, self.update_base_pose)
        rospy.Subscriber('/ee_pose', Point, self.update_ee_pose)
        rospy.Subscriber('/left_hand_pose', Point, self.update_lh_pose)
        rospy.Subscriber('/right_hand_pose', Point, self.update_rh_pose)
        rospy.Subscriber('/cup_pose', Point, self.update_cup_pose)
        rospy.Subscriber('/recognizer/output', String, self.update_speech_input)


    def set_motor_speeds(self, speed_topic, speed):
        rospy.wait_for_service(speed_topic)
        set_speed = rospy.ServiceProxy(speed_topic, dynamixel_controllers.srv.SetSpeed, persistent=True)
        set_speed(speed)

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
        self.init_base_x = base_pose.x
        self.init_base_y = base_pose.y
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

    def update_cup_pose(self, data):
        self.cup_x = data.x
        self.cup_y = data.y

    def update_ee_pose(self, data):
        self.ee_x = data.x
        self.ee_y = data.y
        self.pubvec[4].publish(-0.2)

    def move_to_target(self):
        ee_target_x = self.target_means[self.set_target - 1, 0] + self.init_base_x
        ee_target_y = self.target_means[self.set_target - 1, 1] + self.init_base_y


        self.delta_l = 0.00
        self.theta_command = math.atan2((ee_target_y - self.base_y), (ee_target_x - self.base_x))
        self.l_command = math.sqrt((ee_target_x - self.base_x) ** 2 + (ee_target_y - self.base_y) ** 2) - self.delta_l

        m1_command = self.theta_command - self.thet0
        m3_command = self.full_out + (self.l_command - self.l_max) * \
                                     ((self.len_mid - self.full_out) / (self.l_mid - self.l_max))

        #print('Commands DoF1 : ' + str(m1_command) + ' DoF3 : ' + str(m3_command))

        dist = math.sqrt((ee_target_x - self.ee_x) ** 2 + (ee_target_y - self.ee_y) ** 2)
        lam = 16.252
        self.grip_open_prob = math.exp(-lam*dist*dist)

        if (m1_command > -1.57) and (m1_command < 1.57):
            #print('Moving DoF1')
            self.pubvec[0].publish(m1_command)
        if (m3_command > self.full_out) and (m3_command < self.full_in):
            #print('Moving DoF3')
            self.pubvec[2].publish(m3_command)

        if self.grip_open_prob > 0.80 and (self.pred_state_prev == 'put' or self.prev_cmd == 'put'):
            print('Found target. Opening gripper')
            self.pred_state_prev = 'open'
            self.prev_cmd = 'open'
            self.pubvec[5].publish(self.grip_open)
            self.trial_number += 1
            self.pub_trial_number.publish(self.trial_number)
            self.set_target = 0
            self.pub_speech_command.publish('open')
            self.grip_open_prob = 0.0
            self.speech_current = ''
            #if self.trial_number <= self.task_prediction_threshold:
            #    self.task_predict.add_to_command_buffer(np.array(['open', time.time()], ndmin=2))
            #time.sleep(0.1)

    def training_data_prepare(self):
        tk = time.time()
        if self.speech_current == 'put':
            #print('Updating current data')
            X_current = [self.base_x, self.base_y, self.lh_x, self.lh_y, self.rh_x, self.rh_y]
            Y_current = [2 - (self.trial_number % 2), self.trial_number]  # For first two targets
            if(self.Xkt.size == 0):
                self.Xkt = np.array(X_current, ndmin=2)
                self.Ykt = np.array(Y_current, ndmin=2)
                self.Tnt = np.array(tk, ndmin=2)
            else:
                self.Xkt = np.append(self.Xkt, np.array(X_current, ndmin=2), axis=0)
                self.Ykt = np.append(self.Ykt, np.array(Y_current, ndmin=2), axis=0)
                self.Tnt = np.append(self.Tnt, np.array(tk, ndmin=2), axis=0)

        elif self.speech_current == 'open':
            print('Training Task KNN model')
            self.task_predict.process_data()
            self.task_predict.train()

            if(self.Xnt.size==0):
                self.Xnt = self.Xkt
                self.Ynt = self.Ykt
            else:
                self.Xnt = np.append(self.Xnt, self.Xkt, axis=0)
                self.Ynt = np.append(self.Ynt, self.Ykt, axis=0)
            self.Tnt = np.append(self.Tnt, np.array(tk, ndmin=2), axis=0)
            
            if (self.target_positions.size == 0):
                self.target_positions = np.array([self.ee_x - self.init_base_x, self.ee_y - self.init_base_y], ndmin=2)
            else:
                self.target_positions = np.append(self.target_positions,
                                                   np.array([self.ee_x - self.init_base_x, self.ee_y - self.init_base_y], ndmin=2),
                                                   axis=0)
            
            if self.trial_number >= self.jump_init_number:
                jm = JumpsMethod(self.target_positions)
                jm.Distortions(cluster_range=range(1, self.target_positions.shape[0]), random_state=0)
                jm.Jumps(Y=0.15)

                print('Optimal Number of clusters for N = ' + str(self.target_positions.shape[0]) + ' data points: '
                      + str(jm.recommended_cluster_number))

                self.num_targets = jm.recommended_cluster_number
                if self.num_targets <= 1:
                    self.num_targets = 2
                km = KMeans(n_clusters=self.num_targets)
                self.target_labels = km.fit_predict(self.target_positions) + 1
                self.reprocess_Yn()
                self.compute_target_means()

                print('Training Logit model')
                self.target_logit.assign_data(self.Xnt, self.Ynt[:, 0])
                self.target_logit.train()

            self.trial_number += 1
            self.pub_trial_number.publish(self.trial_number)
            self.speech_current = ''

    def compute_target_means(self):
        labels = np.unique(self.target_labels)
        self.target_means = np.zeros([len(labels), 2])
        for i in range(len(labels)):
            ind = np.where(self.target_labels == labels[i])[0]
            self.target_means[i, :] = np.mean(self.target_positions[ind, :], axis=0)

    def reprocess_Yn(self):
        for i in range(0, int(self.trial_number)):
            ind = np.where(self.Ynt[:, 1].astype(int) == i+1)[0]
            self.Ynt[ind, 0] = self.target_labels[i]

    def update_speech_input(self, dat):
        data = dat.data

        if self.mode == 'train' or self.mode == 'auto' or self.mode == 'speech' or self.mode == 'init':
            self.speech_current = data
            self.pub_speech_command.publish(data)
            if data == 'close':
                print('Gripper Closing')
                self.pubvec[5].publish(self.grip_close)
                self.prev_cmd = 'close'
                time.sleep(0.1)
            elif data == 'open' and self.prev_cmd == 'put':
                print('Gripper Opening')
                self.pubvec[5].publish(self.grip_open)
                self.prev_cmd = 'open'
                time.sleep(0.1)
            elif data == 'go':
                print('Going to handover')
                self.pubvec[5].publish(self.grip_open)
                self.go_to_handover_location()
            elif data == 'put':
                print('Putting away cup')
                self.prev_cmd = 'put'
                self.go_to_dropoff()
            elif data == 'reset':
                self.go_to_init()
                self.prev_cmd = 'reset'
            elif data == 'stop':
                self.stop_motors()
                self.prev_cmd = 'stop'

        if self.mode == 'train':
            if data in self.relevant_commands:
                self.task_predict.add_to_command_buffer(np.array([data, time.time()], ndmin=2))

        elif self.mode == 'auto':
            if data == 'open' and self.prev_cmd == 'put':
                self.trial_number += 1
                self.pub_trial_number.publish(self.trial_number)
                self.pred_state_prev = 'open'
                self.speech_current = ''
            elif data == 'close':
                self.pred_state_prev = 'close'
            elif data == 'go':
                self.pred_state_prev = 'go'

        elif self.mode == 'speech' and data == 'open' and self.prev_cmd == 'put':
            self.trial_number += 1
            self.pub_trial_number.publish(self.trial_number)
            self.speech_current = ''


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

    def go_to_init(self):
        print('Setting motors to initial states')
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(self.initial_angles[i])
            time.sleep(0.5)
        self.pubvec[2].publish(self.full_out)

    def move_after_prediction(self, probs):
        # probs order: [close, go , put]
        thresh = 0.80
        if probs[0] > thresh and self.pred_state_prev == 'go':
            data = 'close'
            msg = rospy.wait_for_message('/cup_pose', Point)
            dist = math.sqrt((msg.x - self.ee_x) ** 2 + (msg.y - self.ee_y) ** 2)
            if dist < 0.25:
                print('Prediction: Gripper closing')
                self.pubvec[5].publish(self.grip_close)
                self.pub_speech_command.publish(data)
                self.pred_state_prev = 'close'
                self.prev_cmd = 'close'
                time.sleep(0.1)
        elif probs[1] > thresh and self.pred_state_prev == 'open':
            data = 'go'
            print('Prediction: Going to handover')
            self.pubvec[5].publish(self.grip_open)
            self.go_to_handover_location()
            self.pub_speech_command.publish(data)
            self.pred_state_prev = 'go'
            self.prev_cmd = 'go'
            time.sleep(0.3)
        elif probs[2] > thresh and self.pred_state_prev == 'close':
            data = 'put'
            self.speech_current = 'put'
            print('Prediction: Putting away cup')
            self.go_to_dropoff()
            self.pred_state_prev = 'put'
            self.prev_cmd = 'put'
            self.pub_speech_command.publish(data)
            #time.sleep(0.3)

    def load_trained_models(self):
        print('Loading trained models')
        self.target_logit.logreg = joblib.load(logit_load_file)
        self.task_predict.knn_model = joblib.load(knn_load_file)

    def save_trained_models(self):
        print('Saving trained models')
        joblib.dump(self.target_logit.logreg, logit_save_file)
        joblib.dump(self.task_predict.knn_model, knn_save_file)

    def training_method(self):
        self.training_data_prepare()
        tk = time.time()
        x_data = np.array(
            [self.base_x, self.base_y, self.ee_x, self.ee_y, self.lh_x, self.lh_y, self.rh_x, self.rh_y, tk],
            ndmin=2)
        self.task_predict.add_to_data_buffer(x_data)

    def autonomous_method(self):
        X_current = [self.base_x, self.base_y, self.lh_x, self.lh_y, self.rh_x, self.rh_y]

        if self.speech_current == 'put':
            self.target_probs = self.target_logit.predict_probabilites(np.array(X_current, ndmin=2))
            probs = self.target_probs.ravel().tolist()
            self.pub_target_probs.publish(Float32MultiArray(data=[100 * x for x in probs]))
            for k in range(len(probs)):
                if probs[k] > 0.8:
                    self.set_target = k + 1
            if self.set_target != 0:
                self.move_to_target()

        x_test = np.array([self.base_x, self.base_y, self.ee_x, self.ee_y, self.lh_x, self.lh_y, self.rh_x, self.rh_y], ndmin=2)
        pred_probs = self.task_predict.predict_probabilites(x_test).ravel()
        probs = [pred_probs[1], pred_probs[2], pred_probs[0], self.grip_open_prob, pred_probs[3]]
        self.move_after_prediction(pred_probs)
        pub_probs = [x * 100 for x in probs]
        self.pub_task_probs.publish(Float32MultiArray(data=pub_probs))

    def speech_method(self):
        pass

    def run(self):
        self.trials_per_condition = 20
        auto_user_input = ''
        speech_user_input = ''
        init_user_input = ''
        order = ''  # After training, ab = autonomous first; ba = speech first

        print('Familiarize user with speech commands before starting study')
        self.mode = 'init'

        while init_user_input != 'Y' and init_user_input != 'y' and init_user_input != 'set':
            init_user_input = raw_input('Begin Training Sequence? (Y/N): ')
            if init_user_input == 'Y' or init_user_input == 'y':
                print('Initiating Training Sequence')
                self.mode = 'train'
                init_user_input = 'set'
            elif init_user_input == 'N' or init_user_input == 'n':
                init_user_input = raw_input('An error had occurred after training. Begin Autonomous Sequence? (Y/N): ')
                if init_user_input == 'Y' or init_user_input == 'y':
                    self.mode = 'auto'
                    self.trial_number = self.trials_per_condition + 1
                    order = 'ab'
                    init_user_input = 'set'
                    self.load_trained_models()
                elif init_user_input == 'N' or init_user_input == 'n':
                    print('Speech mode starting instead')
                    self.mode = 'speech'
                    self.trial_number = self.trials_per_condition + 1
                    order = 'ba'
                    init_user_input = 'set'
                    self.load_trained_models()

        while not rospy.is_shutdown():
            if self.mode == 'train':
                if self.trial_number <= self.trials_per_condition:
                    self.training_method()
                if self.trial_number > self.trials_per_condition:
                    self.save_trained_models()
                    auto_user_input = raw_input('Begin Autonomous Trial First? (Y/N): ')
                    if auto_user_input == 'Y' or auto_user_input == 'y':
                        self.mode = 'auto'
                        order = 'ab'
                    else:
                        speech_user_input = raw_input('Begin Speech Trial First? (Y/N): ')
                        if speech_user_input == 'Y' or speech_user_input == 'y':
                            self.mode = 'speech'
                            order = 'ba'

            elif self.mode == 'auto':
                if order == 'ab':
                    start_cutoff = self.trials_per_condition + 1
                    end_cutoff = 2 * self.trials_per_condition
                    next_mode = 'speech'
                elif order == 'ba':
                    start_cutoff = 2 * self.trials_per_condition + 1
                    end_cutoff = 3 * self.trials_per_condition
                    next_mode = ''
                if self.trial_number == start_cutoff and auto_user_input != 'set':
                    self.mode = ''
                    raw_input('Press any key to initiate Autonomous Trial')
                    self.pred_state_prev = 'open'
                    auto_user_input = 'set'
                    self.mode = 'auto'

                self.autonomous_method()

                if self.trial_number > end_cutoff:
                    self.mode = next_mode
                    if self.mode == '':
                        print('End of Study')

            elif self.mode == 'speech':
                if order == 'ba':
                    start_cutoff = self.trials_per_condition + 1
                    end_cutoff = 2 * self.trials_per_condition
                    next_mode = 'auto'
                elif order == 'ab':
                    start_cutoff = 2 * self.trials_per_condition + 1
                    end_cutoff = 3 * self.trials_per_condition
                    next_mode = ''
                if self.trial_number == start_cutoff and speech_user_input != 'set':
                    self.mode = ''
                    raw_input('Press any key to initiate Speech Trial')
                    speech_user_input = 'set'
                    self.mode = 'speech'

                self.speech_method()

                if self.trial_number > end_cutoff:
                    self.mode = next_mode
                    if self.mode == '':
                        print('End of Study')


if __name__ == '__main__':
    t1 = hrc2d_closed_loop()
    t1.run()
