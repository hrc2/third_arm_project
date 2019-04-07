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
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64, Int32, String

import random
import arbotix_msgs.srv
import dynamixel_msgs.msg
import dynamixel_controllers.srv
from diagnostic_msgs.msg import DiagnosticArray
from apriltags_ros.msg import AprilTagDetectionArray
import csv
from pynput import keyboard
from datetime import date

DATA_PATH = '/home/hriclass/catkin_ws/src/third_arm/scripts/HRC_2D_Task/Data/'
today = str(date.today())
TAGS_FILE = str(DATA_PATH + 'TagData' + today + '.csv')
SPEECH_FILE = str(DATA_PATH + 'SpeechData' + today + '.csv')

class hrc2d_data_logger:
    def __init__(self):
        rospy.init_node('hrc2d_data_logger')
        self.apriltag_topic_list = ['/base_pose', '/ee_pose', '/cup1_pose', '/box1_pose', '/cup2_pose', '/box2_pose',
                                    '/left_hand_pose', '/right_hand_pose']
        #self.speech_command_topic = '/arm_command_state'
        self.tag_ids = [1, 2, 5, 8, 9, 10, 3, 4]
        self.tag_log = np.zeros(2*len(self.tag_ids))
        self.speech_log = ''
        self.header = []

        print('Looking for tag topics')

        for topic in self.apriltag_topic_list:
            dat = rospy.wait_for_message('/base_pose', Point)
            #print(dat)
            self.header.append(str(topic + '_x'))
            self.header.append(str(topic + '_y'))
        print('Found tag topics')

        self.header.append('time')

        speech_header = ['Command', 'Time']
        with open(SPEECH_FILE, 'a') as sf:
            writer = csv.writer(sf)
            writer.writerow(speech_header)

        with open(TAGS_FILE, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(self.header)

    def update_poses(self, data):
        try:
            for i in range(len(self.tag_ids)):
                ind = np.flatnonzero(np.array(self.tag_ids) == data.detections[i].id)
                if not np.isnan(ind):
                    self.tag_log[2 * ind[0]] = data.detections[i].pose.pose.position.x
                    self.tag_log[2 * ind[0] + 1] = data.detections[i].pose.pose.position.y
                    #print(self.tag_log)
            self.print_to_file()

        except (NameError, IndexError):
            return


    def update_speech(self, data):
        self.speech_log = data.data


    def print_to_file(self):
        curr_time = time.time()
        tag_data = np.append(self.tag_log, curr_time)
        speech_data = [self.speech_log, curr_time]

        with open(TAGS_FILE, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(tag_data.tolist())

        with open(SPEECH_FILE, 'a') as sf:
            writer = csv.writer(sf)
            writer.writerow(speech_data)


    def run(self):

        #self.get_initial_states()

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.update_poses)
        rospy.Subscriber('/recognizer/output', String, self.update_speech)
        rospy.spin()

if __name__ == '__main__':
        t1 = hrc2d_data_logger()
        t1.run()