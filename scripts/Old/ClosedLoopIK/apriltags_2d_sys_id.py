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

DOF1_STEP = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/dof1_step.csv'
DOF3_STEP = '/home/hriclass/catkin_ws/src/third_arm/scripts/ClosedLoopIK/data/dof3_step.csv'

class apriltags_sys_id:
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
        self.motor_max_speeds = [1.1, 0.5, 3.0, 0.5, 0.5, 0.5]

        print('Setting motor max speeds')
        for i in range(len(self.motor_max_speeds)):
            self.set_motor_speeds(self.speed_topics[i], self.motor_max_speeds[i])

        #Assume extension starts from full out
        self.get_initial_states()
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
        self.initial_angles = [0.0, 0.0, self.full_in, 0.0, -0.2, 0.0]

        print('Setting motor initial states')
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(self.initial_angles[i])
            time.sleep(2)
        self.t0 = time.time()
        #self.fname = DOF1_STEP
        self.fname = DOF3_STEP
        self.ref = 0.0

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
        self.theta = self.currval[0]
        self.l = self.currval[2]
        self.base_x = base_pose.x
        self.base_y = base_pose.y
        self.ee_x = ee_pose.x
        self.ee_y = ee_pose.y

    def update_theta_state(self, data):
        self.theta = data.current_pos

    def update_l_state(self, data):
        self.l = data.current_pos

    def update_base_pose(self, data):
        self.base_x = data.x
        self.base_y = data.y

    # print("Base- X: " + str(self.base_x) + " Y: " + str(self.base_y))

    def update_ee_pose(self, data):
        self.ee_x = data.x
        self.ee_y = data.y
        # print("Gripper- X: " + str(self.ee_x) + " Y: " + str(self.ee_y))
        #self.closed_loop_control()
        self.write_to_file(self.fname)

    def step_test_1(self):
        for i in range(5):
            self.ref = 0.2
            self.pub_motor1.publish(self.ref)
            time.sleep(2)

            self.ref = 0.0
            self.pub_motor1.publish(self.ref)
            time.sleep(2)

    def step_test_3(self):
        for i in range(5):
            self.ref = self.len_mid
            self.pub_motor3.publish(self.ref)
            time.sleep(2)

            self.ref = self.full_in
            self.pub_motor3.publish(self.ref)
            time.sleep(2)



    def write_to_file(self, fname):
        a = 10

        data = [time.time()-self.t0, self.ref, self.theta, self. l, self.base_x, self.base_y, self.ee_x, self.ee_y]
        data2 = np.array([time.time(), self.ref])
        print(data)

        with open(fname, 'a') as f:
            #np.savetxt(f, data, fmt='%1.4f', delimiter=',')
            datawriter = csv.writer(f, delimiter=',')
            datawriter.writerow(data)

    def run(self):
        # rospy.Subscriber("joy", Joy, self.test_joystick)
        # rospy.Subscriber("joy", Joy, self.get_joystick)
        # rospy.spin()
        self.get_initial_states()

        rospy.Subscriber('/base_swivel_controller/state', dynamixel_msgs.msg.JointState, self.update_theta_state)
        rospy.Subscriber('/arm_extension_controller/state', dynamixel_msgs.msg.JointState, self.update_l_state)
        rospy.Subscriber('/base_pose', Point, self.update_base_pose)
        rospy.Subscriber('/ee_pose', Point, self.update_ee_pose)
        self.step_test_3()
        rospy.spin()



    # self.get_initial_motor_states()
    # self.get_frame_poses()


if __name__ == '__main__':
    t1 = apriltags_sys_id()
    t1.run()