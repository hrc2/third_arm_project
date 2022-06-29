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

from math_funcs import rotm_from_vecs, ik_3d_pos, mocap_filter
from ar_funcs import ar_forecast

class wrf_closed_loop:
    def __init__(self):
        rospy.init_node('wrf_motor_controller')

        self.currval = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command', Float64, queue_size=1)
        self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command', Float64, queue_size=1)
        self.pub_motor4 = rospy.Publisher('/wrist_controller/command', Float64, queue_size=1)
        self.pub_motor5 = rospy.Publisher('/wrist_tilt_controller/command', Float64, queue_size=1)
        self.pub_motor6 = rospy.Publisher('/gripper_controller/command', Float64, queue_size=1)
        self.pub_mocap_filt = rospy.Publisher('/mocap_filtered', Float32MultiArray, queue_size=10)
        self.pubvec = [self.pub_motor1, self.pub_motor2, self.pub_motor3, self.pub_motor4, self.pub_motor5,self.pub_motor6]

        self.motor_max_speeds = [1.5, 1.2, 2.5, 1.5, 1.5, 1.6]
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
        
        self.initial_angles = [0.0, 0.0, self.len_mid, 0.0, -0.2, self.grip_close]
        self.up_lims = [3.1, 0.001, self.full_in - 0.0001, 1.5, 1.5, self.grip_close + 0.1]
        self.lo_lims = [-3.1, -1.9, self.full_out + 0.0001, -1.5, -1.5, self.grip_open - 0.1]
        self.commands = self.initial_angles
        self.cmd_prev = self.commands
        self.d5m = 1

        self.m1_start = 1.8
        self.m2_start = -0.6

        self.data_buffer = np.array([], ndmin=2)
        self.buffer_full = 0
        self.mocap_sample = np.array([], ndmin=2)
        self.mocap_data_filt = np.array([])

        self.curr_mocap_vel = np.array([])
        self.del_t = 1.0/100.0

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
        self.start_motor_loop = 0

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

    def prepare_filter_mocap(self):
        N = 100
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
            self.mocap_sample = self.data_buffer[-S:-1, :]
            
    def filter_mocap(self):

        self.mocap_data_filt, mean_dx = mocap_filter(self.mocap_sample)
             
        self.base_pose = self.mocap_data_filt[-1,0:3].tolist()
        self.ee_pose = self.mocap_data_filt[-1,3:6].tolist()
        self.elbow_pose = self.mocap_data_filt[-1,6:9].tolist()
        self.wrist_pose = self.mocap_data_filt[-1,9:12].tolist()

        self.start_motor_loop = 1   

        self.curr_mocap_vel = mean_dx/self.del_t

        filt_dat = Float32MultiArray()
        filt_dat.data = self.mocap_data_filt[-1,:].tolist()
        self.pub_mocap_filt.publish(filt_dat)        


    def update_base_pose(self,data):
        self.base_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.prepare_filter_mocap()

    def update_ee_pose(self,data):
        self.ee_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        
    def update_wrist_pose(self,data):
        self.wrist_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    def update_elbow_pose(self,data):
        self.elbow_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

        
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
        self.commands = [self.m1_start, self.m2_start, self.len_mid, 0.0, -0.2, self.grip_close]
        self.cmd_prev = self.commands
        print('Setting initial EE pose')        
        self.pub_motor2.publish(self.m2_start)
        time.sleep(3)     
        self.pub_motor1.publish(self.m1_start)
        time.sleep(7)               
        ed = rospy.wait_for_message('/mocap_node/end_eff/pose', PoseStamped)        
        self.ee_init_pose = [ed.pose.position.x, ed.pose.position.y, ed.pose.position.z]
        time.sleep(2)        

    def map_to_commands(self, thets):
        tt1 = thets[0]
        tt2 = thets[1]
        tt3 = thets[2]

        tr1 = -tt1;
        
        tr2 = -2.0*np.pi + 3.99*tt2

        tr3 = self.full_out + (tt3 - self.l_max) * ((self.full_in - self.full_out) / (self.l_min - self.l_max))
        
        tr4 = self.initial_angles[3]
        tr5 = self.initial_angles[4]
        tr6 = self.initial_angles[5]

        return [tr1,tr2,tr3,tr4,tr5,tr6]    

    def control_func_ar(self):

        elbow_vel = np.linalg.norm(self.curr_mocap_vel[0:3])
        wrist_vel = np.linalg.norm(self.curr_mocap_vel[9:12])
        thresh = 0.005
        pred_horizon = 6
        ar_order = 18

        pred_on = False

        control_coeffs = np.logspace(0, 0, num=pred_horizon)

        if (elbow_vel >= thresh or wrist_vel >= thresh) and pred_on == True:
            ar_sample = self.mocap_sample[-ar_order:, :]
            base_pred, elbow_pred, wrist_pred = ar_forecast(ar_sample, pred_horizon)
            init_cmd = 0

            for i in range(pred_horizon):
                #elb_vec = np.array(wrist_pred[i,:]) - np.array(elbow_pred[i,:])
                elb_vec = np.array(wrist_pred[i,:]) - np.array(base_pred[i,:])
                R_elb = rotm_from_vecs(np.array([1,0,0]),np.array(elb_vec))
                self.delta_p = np.dot(R_elb.transpose(), np.array(self.ee_init_pose) - np.array(base_pred[i,:]))
                thets = ik_3d_pos(self.delta_p)
                if i == 0:
                    init_cmd = np.array(self.map_to_commands(thets))       
                new_cmd = np.array(self.map_to_commands(thets))
                self.cmd_prev = self.commands
                cmd_compute = init_cmd + control_coeffs[i]*(new_cmd - init_cmd)
                self.commands = cmd_compute.tolist()
                self.send_cmd_to_motor()           
                #print('AR pred:', [base_pred[i,:], elbow_pred[i,:], wrist_pred[i,:]])
                #print('Forecast Command:', self.commands)
        else:
            #elb_vec = np.array(self.wrist_pose) - np.array(self.elbow_pose)
            elb_vec = np.array(self.wrist_pose) - np.array(self.base_pose)
            R_elb = rotm_from_vecs(np.array([1,0,0]),np.array(elb_vec))
            self.delta_p = np.dot(R_elb.transpose(), np.array(self.ee_init_pose) - np.array(self.base_pose))
            thets = ik_3d_pos(self.delta_p)          
            self.cmd_prev = self.commands
            self.commands = self.map_to_commands(thets) 
            self.send_cmd_to_motor()
        
    def send_cmd_to_motor(self):
        for i in range(len(self.commands)):
            max_delta = 0.15*np.abs(self.lo_lims[i] - self.up_lims[i])
            if self.commands[i] <= self.up_lims[i] and self.commands[i] >= self.lo_lims[i]:                
                if i == 0:
                    max_delta = 0.2
                    if np.abs(self.commands[i] - self.m1_start) <= max_delta:
                        self.pubvec[i].publish(self.commands[i])     
                elif np.abs(self.commands[i] - self.cmd_prev[i]) <= max_delta:
                    self.pubvec[i].publish(self.commands[i])           

    def run(self):
        while not rospy.is_shutdown():
            if self.mocap_sample.size != 0:
                self.filter_mocap()
            if self.start_motor_loop == 1:
                self.control_func_ar()
        rospy.spin()

if __name__ == '__main__':
    node = wrf_closed_loop()
    node.run()
