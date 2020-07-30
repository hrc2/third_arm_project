#!/usr/bin/env python
import rospy

import math
import time
import numpy as np
import scipy.signal as sg
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

from math_funcs import rotm_from_vecs, ik_3d_pos

class wrf_sys_id:
    def __init__(self):
        rospy.init_node('wrf_motor_controller')

        self.currval = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command', Float64, queue_size=1)
        self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command', Float64, queue_size=1)
        self.pub_motor4 = rospy.Publisher('/wrist_controller/command', Float64, queue_size=1)
        self.pub_motor5 = rospy.Publisher('/wrist_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor6 = rospy.Publisher('/gripper_controller/command', Float64, queue_size=1)
        self.pubvec = [self.pub_motor1, self.pub_motor2, self.pub_motor3, self.pub_motor4, self.pub_motor5,self.pub_motor6]

        self.motor_max_speeds = [2.0, 1.2, 2.5, 1.5, 1.5, 1.6]
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
        
        self.full_in = self.full_out + 1.3
        self.len_mid = 0.5*(self.full_out + self.full_in)
        self.l_max = 0.45
        self.l_min = 0.33
        if self.len_mid < self.full_out and self.len_mid > self.full_in:
            print('Error: Re-calibrate length extension')
            exit(0)        
        
        self.initial_angles = [0.0, 0.0, self.len_mid, 0.0, -0.7, self.grip_close]
        self.up_lims = [3.1, 0.001, self.full_in - 0.0001, 1.5, 1.5, self.grip_close + 0.0001]
        self.lo_lims = [-3.1, -4.5, self.full_out + 0.0001, 1.5, 1.5, self.grip_open - 0.0001]
        self.commands = self.initial_angles
        self.d5m = 1

        self.data_buffer = np.array([], ndmin=2)
        self.buffer_full = 0

        print('Setting motor initial states')
        for i in range(len(self.initial_angles)):
            self.pubvec[i].publish(self.initial_angles[i])
            time.sleep(1.5)

        rospy.Subscriber('/base_swivel_controller/state', dynamixel_msgs.msg.JointState, self.update_t1)
        rospy.Subscriber('/vertical_tilt_controller/state', dynamixel_msgs.msg.JointState, self.update_t2)
        rospy.Subscriber('/arm_extension_controller/state', dynamixel_msgs.msg.JointState, self.update_t3)
        rospy.Subscriber('/wrist_controller/state', dynamixel_msgs.msg.JointState, self.update_t4)
        rospy.Subscriber('/wrist_tilt_controller/state', dynamixel_msgs.msg.JointState, self.update_t5)
        rospy.Subscriber('/gripper_controller/state', dynamixel_msgs.msg.JointState, self.update_t6)
        
        rospy.Subscriber('/mocap_node/wrf_base/pose', PoseStamped, self.update_base_pose)
        rospy.Subscriber('/mocap_node/end_eff/pose', PoseStamped, self.update_ee_pose)
        rospy.Subscriber('/mocap_node/hand/pose', PoseStamped, self.update_wrist_pose)
        rospy.Subscriber('/mocap_node/elbow/pose', PoseStamped, self.update_elbow_pose)


        self.initialize()

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
            self.pub_motor4.publish(self.initial_angles[3])    
    def update_t6(self,data):
        self.currval[5] = data.current_pos

    def filter_mocap(self):
        # Transfer function for IIR low-pass filters, cutoff at 12 Hz
        b = np.array([0.1400982208, -0.0343775491, 0.0454003083, 0.0099732061, 0.0008485135])
        a = np.array([1, -1.9185418203, 1.5929378702, -0.5939699187, 0.0814687111])
        N = 50
        S = 20
        
        newdat = np.ravel(np.array([self.base_pose, self.ee_pose, self.elbow_pose, self.wrist_pose]))
        newdat = newdat.tolist()

        if self.buffer_full == 0:
            if self.data_buffer.size == 0:                
                self.data_buffer = np.array(newdat, ndmin=2)
                #print('First pose: ', self.data_buffer)                
            else:                
                self.data_buffer = np.append(self.data_buffer, np.array(newdat, ndmin=2), axis=0)                                
                if self.data_buffer.shape[0] >= N:
                    self.buffer_full = 1
        else:
            buff_del = np.delete(self.data_buffer, 0, 0)
            self.data_buffer = np.append(buff_del, np.array(newdat, ndmin=2), axis=0)                        
            sample = self.data_buffer[-S:-1, :]
            data_filt = sg.filtfilt(b, a, x=sample, axis=0)
            #print('Filtered pose: ', data_filt[-1,:])
            self.base_pose = data_filt[-1,0:3].tolist()
            self.base_pose = data_filt[-1,3:6].tolist()
            self.base_pose = data_filt[-1,6:9].tolist()
            self.base_pose = data_filt[-1,9:12].tolist()
            #time.sleep(2)


    def update_base_pose(self,data):
        self.base_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.filter_mocap()
    def update_ee_pose(self,data):
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        #self.filter_mocap()
    def update_wrist_pose(self,data):
        self.wrist_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        #self.filter_mocap()
    def update_elbow_pose(self,data):
        self.elbow_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        #self.filter_mocap()
        
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
            data = rospy.wait_for_message(topic, dynamixel_msgs.msg.JointState)
            self.currval[self.count] = data.current_pos
            print("Topic name : " + topic + " Angle : " + str(data.current_pos))
            self.count += 1

        time.sleep(2)    

        bd = rospy.wait_for_message('/mocap_node/wrf_base/pose', PoseStamped)
        ed = rospy.wait_for_message('/mocap_node/end_eff/pose', PoseStamped)
        hd = rospy.wait_for_message('/mocap_node/hand/pose', PoseStamped)
        eld = rospy.wait_for_message('/mocap_node/elbow/pose', PoseStamped)
        self.base_pose = [bd.pose.position.x, bd.pose.position.y, bd.pose.position.z]
        self.ee_pose = [ed.pose.position.x, ed.pose.position.y, ed.pose.position.z]
        self.wrist_pose = [hd.pose.position.x, hd.pose.position.y, hd.pose.position.z]
        self.elbow_pose = [eld.pose.position.x, eld.pose.position.y, eld.pose.position.z]

        print('Detected Mocap Poses')
        time.sleep(2)
    
    def initialize(self):
        print('Setting initial EE pose')        
        self.pub_motor2.publish(-2.0)
        time.sleep(3)     
        self.pub_motor1.publish(1.5)
        time.sleep(7)               
        ed = rospy.wait_for_message('/mocap_node/end_eff/pose', PoseStamped)        
        self.ee_init_pose = [ed.pose.position.x, ed.pose.position.y, ed.pose.position.z]
        time.sleep(2)        

    def map_to_commands(self, thets):
        tt1 = thets[0]
        tt2 = thets[1]
        tt3 = thets[2]

        tr1 = -tt1;
        
        tr2 = -2*np.pi + 4*tt2

        tr3 = self.full_out + (tt3 - self.l_max) * ((self.full_in - self.full_out) / (self.l_min - self.l_max))
        
        tr4 = self.initial_angles[3]
        tr5 = self.initial_angles[4]
        tr6 = self.initial_angles[5]

        self.commands = [tr1,tr2,tr3,tr4,tr5,tr6]

    def send_cmd_to_motor(self):
        for i in range(len(self.commands)):
            if self.commands[i] <= self.up_lims[i] and self.commands[i] >= self.lo_lims[i]:
                self.pubvec[i].publish(self.commands[i])          

        
    def closed_loop(self):
        print('Starting closed loop control')
        
        elb_vec = np.array(self.wrist_pose) - np.array(self.elbow_pose)
        R_elb = rotm_from_vecs(np.array([1,0,0]),np.array(elb_vec))

        self.delta_p = np.dot(R_elb.transpose(), np.array(self.ee_init_pose) - np.array(self.base_pose))
        thets = ik_3d_pos(self.delta_p)

        self.map_to_commands(thets)      

        #print('Desired Pose from IK:', thets)
        #print('Motor Commands:', self.commands)

        self.send_cmd_to_motor()


    def run(self):
        while not rospy.is_shutdown():
            self.closed_loop()
        rospy.spin()

if __name__ == '__main__':
    t1 = wrf_sys_id()
    t1.run()
