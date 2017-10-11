#!/usr/bin/env python

import pygame
from pygame.locals import *
from sys import exit
from bluepy import btle
import math
import struct
#import BB8_driver
#from sphero_driver import sphero_driver
#from sphero import SpheroNode

import rospy

import math
import numpy as np
import sys
import tf
import PyKDL
import dynamic_reconfigure.server
import argparse

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from std_msgs.msg import ColorRGBA, Float32, Bool



class thirdarm_joystick:    

    self.currval = [0.0, 0.0, 0.0, 0.0, 0.0]
    self.command = [0.0, 0.0, 0.0, 0.0, 0.0]

    pygame.joystick.init()
    self.j1 = pygame.joystick.Joystick(0)

    def __init__(self):
        rospy.init_node('thirdarm_joystick')
        self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command',Float64, queue_size=1)
        self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command',Float64, queue_size=1)
        self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command',Float64, queue_size=1)
        self.pub_motor4 = rospy.Publisher('/wrist_controller/command',Float64, queue_size=1)
        self.pub_motor5 = rospy.Publisher('/gripper_controller/command',Float64, queue_size=1)

        self.pubvec = [self.pub_motor1,self.pub_motor2,self.pub_motor3,self.pub_motor4,self.pub_motor5 ]
    
    def arm_callback(self, data):

        for i in range(size(self.currval)):
            self.currval[i] = data.motor_states[i].position

        self.get_joystick() 
        
        

    def get_joystick():
        # Axis, buttons in use:
        # Axis 0: D-Pad Left Right
        # Axis 1: D-Pad Up Down
        # Axis 2: Right Analog Left Right
        # Axis 3: Right Analog Up Down
        # Axis 4: Left Analog Left Right
        # Axis 5: Left Analog Up Down
        # Button 0: Blue X

        # Mapping to DoFs:
        # Dof1: Base Swivel: Axis 2
        # Dof2: Vertical pitching: Axis 3
        # Dof3: Length extension: Axis 5
        # Dof4: Wrist rotation: Axis 0
        # Gripper: Button 0


        c1 = self.j1.get_axis(2)
        c2 = self.j1.get_axis(3)
        c3 = self.j1.get_axis(5)
        c4 = self.j1.get_axis(0)
        c5 = self.j1.get_button(0)
        
        # Assign commands for each Dof:

        cvec = [c1,c2,c3,c4,c5]
        cmd_scale = [0.2, 0.2, 0.1, 0.3, 1]

        grip_close = 0.6
        grip_open = 1.6

        for i in range(size(cvec)):
            if not i == 4:
                self.command[i] = self.currval[i] + np.sign(cvec[i])*cmd_scale[i]
            else if cvec[i] == True:
                if math.fabs(self.currval[i] - grip_close)<0.2:
                    self.command[i] = grip_open
                else:
                    self.command[i] = grip_close

        self.publish_cmd()

    def publish_cmd(self):
        for i in range(size(self.pubvec)):
            pubvec[i].publish(self.command[i])

    def execute(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.motor_sub = rospy.Subscriber('/motor_states/third_arm_port', dynamixel_msgs.msg.MotorStateList, self.arm_callback)           
            r.sleep()
        


if __name__ == '__main__':
    t1 = thirdarm_joystick()
    t1.execute()
    