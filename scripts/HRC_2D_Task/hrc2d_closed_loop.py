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
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64, Int32

import random
import arbotix_msgs.srv
import dynamixel_msgs.msg
import dynamixel_controllers.srv
from diagnostic_msgs.msg import DiagnosticArray
from apriltags_ros.msg import AprilTagDetectionArray
import csv
from pynput import keyboard

class hrc2d_closed_loop:
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

        #self.pub_within_range = rospy.Publisher('/within_range', Int32, queue_size=1)

        self.pubvec = [self.pub_motor1, self.pub_motor2, self.pub_motor3, self.pub_motor4, self.pub_motor5,
                       self.pub_motor6]


        self.speed_topics = ['base_swivel_controller/set_speed', '/vertical_tilt_controller/set_speed',
                             '/arm_extension_controller/set_speed', '/wrist_controller/set_speed',
                             '/wrist_tilt_controller/set_speed', '/gripper_controller/set_speed']
        self.motor_max_speeds = [0.5, 0.5, 2.0, 0.5, 0.5, 0.5]

        print('Setting motor max speeds')
        for i in range(len(self.motor_max_speeds)):
            self.set_motor_speeds(self.speed_topics[i], self.motor_max_speeds[i])

        #Assume extension starts from full out
        self.get_initial_states()

        self.grip_open = 0.0
        self.grip_close = 1.2

        self.full_out = self.currval[2]
        if self.full_out < 0.2:
            print('Error: Re-calibrate length extension')
            exit(0)
        #self.full_out = self.full_in - 2.0
        self.full_in = self.full_out + 2.0
        self.len_mid = 0.5*(self.full_out + self.full_in)
        if self.len_mid < self.full_out and self.len_mid > self.full_in:
            print('Error: Re-calibrate length extension')
            exit(0)

        #self.initial_angles = [0.0, -0.8, 0.5*(self.full_in + self.full_out), 0.0, -0.64, 0.0]
        self.initial_angles = [0.0, 0.0, self.len_mid, 0.0, -0.2, 0.0]

        print('Setting motor initial states')
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(self.initial_angles[i])
            time.sleep(2)

        self.calibrate()


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
        cup1_pose = rospy.wait_for_message('/cup1_pose', Point)
        self.theta = self.currval[0]
        self.l = self.currval[2]
        self.base_x = base_pose.x
        self.base_y = base_pose.y
        self.ee_x = ee_pose.x
        self.ee_y = ee_pose.y
        self.cup1_x = cup1_pose.x
        self.cup1_y = cup1_pose.y

    def calibrate(self):
        self.thet0 = math.atan2((self.ee_y - self.base_y), (self.ee_x - self.base_x))
        self.l_mid = math.sqrt((self.base_x - self.ee_x) ** 2 + (self.base_y - self.ee_y) ** 2)

        self.pubvec[2].publish(self.full_out)
        time.sleep(2)
        ee_pose = rospy.wait_for_message('/ee_pose', Point)
        self.ee_x = ee_pose.x
        self.ee_y = ee_pose.y
        self.l_max = math.sqrt((self.base_x - self.ee_x) ** 2 + (self.base_y - self.ee_y) ** 2)

        print('Calibration Done')


    def update_theta_state(self, data):
        self.theta = data.current_pos

    def update_l_state(self, data):
        self.l = data.current_pos

    def update_base_pose(self, data):
        self.base_x = data.x
        self.base_y = data.y

    def update_cup1_pose(self, data):
        self.cup1_x = data.x
        self.cup1_y = data.y

    def update_ee_pose(self, data):
        self.ee_x = data.x
        self.ee_y = data.y
        self.pubvec[4].publish(-0.2)

    def on_press(self, key):
        try:
            k = key.char
        except:
            k = key.name

        if key == keyboard.Key.esc:
            print('Stopping Keyboard Listener')
            return False

        if k == 'g':
            print('Gripper Closing')
            self.pubvec[5].publish(self.grip_close)
            time.sleep(0.1)
        elif k == 'j':
            print('Gripper Opening')
            self.pubvec[5].publish(self.grip_open)
            time.sleep(0.1)
        elif k == 'c':
            self.go_to_cup()
        elif k == 'n':
            self.go_to_init()

    def go_to_cup(self):
        self.theta_command = math.atan2((self.cup1_y - self.base_y), (self.cup1_x - self.base_x))
        self.l_command = math.sqrt((self.cup1_x - self.base_x) ** 2 + (self.cup1_y - self.base_y) ** 2)

        m1_command = self.theta_command - self.thet0
        m3_command = self.full_out + (self.l_command - self.l_max) * (
        (self.len_mid - self.full_out) / (self.l_mid - self.l_max))

        print ('Commands DoF1 : ' + str(m1_command) + ' DoF3 : ' + str(m3_command))

        if (m1_command > -1.57) and (m1_command < 1.57):
            print('Moving DoF1')
            self.pubvec[0].publish(m1_command)

    def go_to_init(self):
        print('Setting motors to initial states')
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(self.initial_angles[i])
            time.sleep(0.5)

    def run(self):

        #self.get_initial_states()

        rospy.Subscriber('/base_swivel_controller/state', dynamixel_msgs.msg.JointState, self.update_theta_state)
        rospy.Subscriber('/arm_extension_controller/state', dynamixel_msgs.msg.JointState, self.update_l_state)
        rospy.Subscriber('/base_pose', Point, self.update_base_pose)
        rospy.Subscriber('/ee_pose', Point, self.update_ee_pose)
        rospy.Subscriber('/cup1_pose', Point, self.update_cup1_pose)

        kb_lis = keyboard.Listener(on_press=self.on_press)
        kb_lis.start()
        kb_lis.join()

        rospy.spin()



    # self.get_initial_motor_states()
    # self.get_frame_poses()


if __name__ == '__main__':
    t1 = hrc2d_closed_loop()
    t1.run()
