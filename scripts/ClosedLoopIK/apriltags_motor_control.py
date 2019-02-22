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

        self.pub_within_range = rospy.Publisher('/within_range', Int32, queue_size=1)

        self.pubvec = [self.pub_motor1, self.pub_motor2, self.pub_motor3, self.pub_motor4, self.pub_motor5,
                       self.pub_motor6]


        self.speed_topics = ['base_swivel_controller/set_speed', '/vertical_tilt_controller/set_speed',
                             '/arm_extension_controller/set_speed', '/wrist_controller/set_speed',
                             '/wrist_tilt_controller/set_speed', '/gripper_controller/set_speed']
        self.motor_max_speeds = [1.0, 0.5, 3.0, 0.5, 0.5, 0.5]

        print('Setting motor max speeds')
        for i in range(len(self.motor_max_speeds)):
            self.set_motor_speeds(self.speed_topics[i], self.motor_max_speeds[i])

        #Assume extension starts from full in
        self.get_initial_motor_states()
        self.full_in = self.currval[2]
        if self.full_in < 1.8:
            print('Error: Re-calibrate length extension')
            exit(0)
        self.full_out = self.full_in - 1.7
        self.len_mid = 0.5*(self.full_out + self.full_in)
        if self.len_mid < self.full_out and self.len_mid > self.full_in:
            print('Error: Re-calibrate length extension')
            exit(0)
        self.within_range = 0

        #self.initial_angles = [0.0, -0.8, 0.5*(self.full_in + self.full_out), 0.0, -0.64, 0.0]
        self.initial_angles = [0.0, 0.0, self.full_in, 0.0, -0.2, 0.0]

        print('Setting motor initial states')
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(self.initial_angles[i])
            time.sleep(1)

    def set_motor_speeds(self, speed_topic, speed):
        self.set_speed = rospy.ServiceProxy(speed_topic, dynamixel_controllers.srv.SetSpeed)
        self.set_speed(speed)

    def get_initial_motor_states(self):
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

    def get_initial_frame_poses(self):
        t0 = time.time()
        t = time.time()
        timeout = 2.0
        print("Setting initial pose values: " + str(timeout) + " seconds averaged")
        self.pubvec[2].publish(self.len_mid)
        time.sleep(1)
        self.base_x_avg = 0.0
        self.base_y_avg = 0.0
        self.ee_x_avg = 0.0
        self.ee_y_avg = 0.0
        count = 0
        while (t < t0 + timeout):
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


        self.thet0 = math.atan2((self.ee_y_avg - self.base_y_avg), (self.ee_x_avg - self.base_x_avg))
        self.length_calibrate()
        self.pubvec[2].publish(self.len_mid)
        time.sleep(1)


    def update_theta_state(self, data):
        self.theta = data

    def update_l_state(self, data):
        self.l = data

    def update_base_pose(self, data):
        self.base_x = data.x
        self.base_y = data.y

    # print("Base- X: " + str(self.base_x) + " Y: " + str(self.base_y))

    def update_ee_pose(self, data):
        self.ee_x = data.x
        self.ee_y = data.y
        # print("Gripper- X: " + str(self.ee_x) + " Y: " + str(self.ee_y))
        self.closed_loop_control()

    def length_calibrate(self):

        t0 = time.time()
        t = time.time()
        timeout = 2.0
        print("Calibrating length: " + str(timeout) + " seconds averaged")
        self.pubvec[2].publish(self.full_in)
        time.sleep(1)
        ee_x_avg = 0.0
        ee_y_avg = 0.0
        count = 0
        while (t < t0 + timeout):
            ee_pose = rospy.wait_for_message('/ee_pose', Point)
            ee_x_avg += ee_pose.x
            ee_y_avg += ee_pose.y
            count += 1
            t = time.time()
        ee_x_avg /= count
        ee_y_avg /= count
        self.l0 = math.sqrt((self.base_x_avg - ee_x_avg) ** 2 + (self.base_y_avg - ee_y_avg) ** 2)

        self.pubvec[2].publish(self.full_out)
        time.sleep(1)
        ee_x_avg = 0.0
        ee_y_avg = 0.0
        count = 0
        t0 = time.time()
        while (t < t0 + timeout):
            ee_pose = rospy.wait_for_message('/ee_pose', Point)
            ee_x_avg += ee_pose.x
            ee_y_avg += ee_pose.y
            count += 1
            t = time.time()
        ee_x_avg /= count
        ee_y_avg /= count

        self.l_max = math.sqrt((self.base_x_avg - ee_x_avg) ** 2 + (self.base_y_avg - ee_y_avg) ** 2)


    def closed_loop_control(self):
        # print('Closed Loop Controller')
        self.del_x = self.base_x - self.base_x_avg
        self.del_y = self.base_y - self.base_y_avg

        self.theta_command = math.atan2((self.ee_y_avg - self.base_y), (self.ee_x_avg - self.base_x))
        self.l_command = math.sqrt((self.ee_x_avg - self.base_x) ** 2 + (self.ee_y_avg - self.base_y) ** 2)
        # print("Theta Change : " + str(self.theta_command - self.thet0))
        # print("Length Change : " + str(self.l_command - self.l0))

        m1_command = self.theta_command - self.thet0
        m3_command = self.full_out + (self.l_command - self.l_max) * (
        (self.full_in - self.full_out) / (self.l0 - self.l_max))

        if math.fabs(self.del_x) > 0.001 and math.fabs(self.del_y) > 0.001:
            print("Base command : " + str(m1_command))
            print("Extension command : " + str(m3_command))
            self.pubvec[0].publish(m1_command)
            if m3_command <= self.full_in and m3_command >= self.full_out:
                #self.pubvec[0].publish(m1_command)
                self.pubvec[2].publish(m3_command)
                self.within_range = 1
            else:
                self.within_range = 0

        self.pub_within_range.publish(self.within_range)
            # print("Del X : " + str(self.del_x))
            # print("Del Y : " + str(self.del_y))

    def run(self):
        # rospy.Subscriber("joy", Joy, self.test_joystick)
        # rospy.Subscriber("joy", Joy, self.get_joystick)
        # rospy.spin()
        self.get_initial_motor_states()
        self.get_initial_frame_poses()
        self.closed_loop_control()
        rospy.Subscriber('/base_swivel_controller/state', dynamixel_msgs.msg.JointState, self.update_theta_state)
        rospy.Subscriber('/arm_extension_controller/state', dynamixel_msgs.msg.JointState, self.update_l_state)

        rospy.Subscriber('/base_pose', Point, self.update_base_pose)
        rospy.Subscriber('/ee_pose', Point, self.update_ee_pose)
        rospy.spin()

    # self.get_initial_motor_states()
    # self.get_frame_poses()


if __name__ == '__main__':
    t1 = apriltags_motor_control()
    t1.run()
