#!/usr/bin/env python
import rospy

import math
import time
import numpy as np
import sys
import tf
from datetime import date

import sensor_msgs.msg
from sensor_msgs.msg import Imu, JointState, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3, PoseStamped
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64, Int32, String, Float32MultiArray

import arbotix_msgs.srv
import dynamixel_msgs.msg
import dynamixel_controllers.srv
from diagnostic_msgs.msg import DiagnosticArray
import csv

class wrf_sys_id:
    def __init__(self):
        rospy.init_node('wrf_motor_controller')

        self.currval = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command', Float64, queue_size=1)
        self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command', Float64, queue_size=1)
        self.pub_motor4 = rospy.Publisher('/wrist_controller/command', Float64, queue_size=1)
        self.pub_motor5 = rospy.Publisher('/wrist_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor6 = rospy.Publisher('/gripper_controller/command', Float64, queue_size=1)
        self.pubvec = [self.pub_motor1, self.pub_motor2, self.pub_motor3, self.pub_motor4, self.pub_motor5,self.pub_motor6]

        self.motor_max_speeds = [2.0, 1.5, 2.5, 1.5, 1.5, 1.6]
        self.speed_topics = ['/base_swivel_controller/set_speed', '/vertical_tilt_controller/set_speed',
                             '/arm_extension_controller/set_speed', '/wrist_controller/set_speed',
                             '/wrist_tilt_controller/set_speed', '/gripper_controller/set_speed']

        print('Setting motor max speeds')
        for i in range(len(self.motor_max_speeds)):
            self.set_motor_speeds(self.speed_topics[i], self.motor_max_speeds[i])

        #Assume extension starts from full out
        self.get_initial_states()

        self.grip_open = 0.0
        self.grip_close = 1.2

        self.d1_handover = 0.4
        self.d1_dropoff = -1.8

        self.full_out = self.currval[2]
        if self.full_out < 0.2:
            print('Error: Re-calibrate length extension')
            exit(0)
        
        self.full_in = self.full_out + 1.2
        self.len_mid = 0.5*(self.full_out + self.full_in)
        if self.len_mid < self.full_out and self.len_mid > self.full_in:
            print('Error: Re-calibrate length extension')
            exit(0)        
        
        self.initial_angles = [0.0, 0.0, self.full_out, 0.0, -0.1, self.grip_close]
        self.d5m = 1

        print('Setting motor initial states')
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(self.initial_angles[i])
            time.sleep(1)

        rospy.Subscriber('/base_swivel_controller/state', dynamixel_msgs.msg.JointState, self.update_t1)
        rospy.Subscriber('/vertical_tilt_controller/state', dynamixel_msgs.msg.JointState, self.update_t2)
        rospy.Subscriber('/arm_extension_controller/state', dynamixel_msgs.msg.JointState, self.update_t3)
        rospy.Subscriber('/wrist_controller/state', dynamixel_msgs.msg.JointState, self.update_t4)
        rospy.Subscriber('/wrist_tilt_controller/state', dynamixel_msgs.msg.JointState, self.update_t5)
        rospy.Subscriber('/gripper_controller/state', dynamixel_msgs.msg.JointState, self.update_t6)
        
        rospy.Subscriber('/mocap_node/wrf_base/pose', PoseStamped, self.update_base_pose)
        rospy.Subscriber('/mocap_node/end_eff/pose', PoseStamped, self.update_ee_pose)

        self.length_calibrate()

    def update_t1(self,data):
        self.currval[0] = data.current_pos    
    def update_t2(self,data):
        self.currval[1] = data.current_pos    
    def update_t3(self,data):
        self.currval[2] = data.current_pos    
    def update_t4(self,data):
        self.currval[3] = data.current_pos    
    def update_t5(self,data):
        self.currval[4] = data.current_pos
        if self.d5m == 1:
            self.pub_motor5.publish(self.initial_angles[4])    
    def update_t6(self,data):
        self.currval[5] = data.current_pos

    def update_base_pose(self,data):
        self.base_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
    def update_ee_pose(self,data):
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        
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

        time.sleep(2)    

        bd = rospy.wait_for_message('/mocap_node/wrf_base/pose', PoseStamped)
        ed = rospy.wait_for_message('/mocap_node/end_eff/pose', PoseStamped)

        print('Detected Mocap Poses')
    
    def length_calibrate(self):
        print('Calibrating Length for DoF 3')
        
        self.pub_motor3.publish(self.full_out)
        time.sleep(2)
        bd = rospy.wait_for_message('/mocap_node/wrf_base/pose', PoseStamped)
        ed = rospy.wait_for_message('/mocap_node/end_eff/pose', PoseStamped)
        bp = np.array([bd.pose.position.x, bd.pose.position.y, bd.pose.position.z])
        ep = np.array([ed.pose.position.x, ed.pose.position.y, ed.pose.position.z])
        self.l_max = np.linalg.norm(ep-bp)

        self.pub_motor3.publish(self.full_in)
        time.sleep(2)
        bd = rospy.wait_for_message('/mocap_node/wrf_base/pose', PoseStamped)
        ed = rospy.wait_for_message('/mocap_node/end_eff/pose', PoseStamped)
        bp = np.array([bd.pose.position.x, bd.pose.position.y, bd.pose.position.z])
        ep = np.array([ed.pose.position.x, ed.pose.position.y, ed.pose.position.z])
        self.l_min = np.linalg.norm(ep-bp)

        self.pub_motor3.publish(self.len_mid)
        time.sleep(2)
        bd = rospy.wait_for_message('/mocap_node/wrf_base/pose', PoseStamped)
        ed = rospy.wait_for_message('/mocap_node/end_eff/pose', PoseStamped)
        bp = np.array([bd.pose.position.x, bd.pose.position.y, bd.pose.position.z])        
        self.ee_init = [ed.pose.position.x, ed.pose.position.y, ed.pose.position.z]


    def move_dof_1(self):
        print('Starting Dof-1 step test')
        time.sleep(5)
        for i in range(10):
            self.pub_motor1.publish(0.4)
            time.sleep(3)
            self.pub_motor1.publish(0.0)
            time.sleep(3)

    def dof_3_closed_loop(self):
        self.l_command = np.linalg.norm(np.array(self.ee_init) - np.array(self.base_pose))
        
        m3_command = self.full_out + (self.l_command - self.l_max) * (
        (self.full_in - self.full_out) / (self.l_min - self.l_max))

        print('Dof-3 Full in:', self.full_in)
        print('Dof-3 Full out:', self.full_out)
        print('Dof-3 command:', m3_command)

        if m3_command <= self.full_in and m3_command >= self.full_out:
            self.pub_motor3.publish(m3_command)   

    def run(self):
        while not rospy.is_shutdown():
            self.dof_3_closed_loop()
        rospy.spin()

if __name__ == '__main__':
    t1 = wrf_sys_id()
    t1.run()
