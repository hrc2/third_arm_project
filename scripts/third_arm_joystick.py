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
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64

#import rospy
#import roslib
import random
#import math
import arbotix_msgs.srv
#from std_msgs.msg import String, Float64
#from geometry_msgs.msg import Vector3, Twist
#from sensor_msgs.msg import JointState
import dynamixel_msgs.msg
#import numpy as np
import csv


class thirdarm_joystick:    

    currval = [0.0, 0.0, 0.0, 0.0, 0.0]
    command = [0.0, 0.0, 0.0, 0.0, 0.0]

    

    def __init__(self):
        rospy.init_node('thirdarm_joystick')
        
        pygame.init()
        pygame.joystick.init()
        self.j1 = pygame.joystick.Joystick(0)
        self.j1.init()


        #print pygame.joystick.get_count()
        

        self.pub_motor1 = rospy.Publisher('/base_swivel_controller/command',Float64, queue_size=1)
        self.pub_motor2 = rospy.Publisher('/vertical_tilt_controller/command',Float64, queue_size=1)
        self.pub_motor3 = rospy.Publisher('/arm_extension_controller/command',Float64, queue_size=1)
        self.pub_motor4 = rospy.Publisher('/wrist_controller/command',Float64, queue_size=1)
        self.pub_motor5 = rospy.Publisher('/gripper_controller/command',Float64, queue_size=1)

        self.pubvec = [self.pub_motor1,self.pub_motor2,self.pub_motor3,self.pub_motor4,self.pub_motor5 ]
    
    def arm_callback(self, data):

        for i in range(len(self.currval)):
            self.currval[i] = data.motor_states[i].position

        self.get_joystick() 
        
        

    def get_joystick(self):
        pygame.event.get()
        # Axis, buttons in use:
        # Axis 0: D-Pad Left Right
        # Axis 1: D-Pad Up Down
        # Axis 2: Right Analog Left Right
        # Axis 3: Right Analog Up Down
        # Axis 4: Left Analog Left Right
        # Axis 5: Left Analog Up Down
        # Button 0: Blue X

        # Mapping to DoFs:
        # Dof1: Base Swivel: Axis 2 : Right analog L/R
        # Dof2: Vertical pitching: Hat 0 : Left analog U/D
        # Dof3: Length extension: Axis 1 : D-Pad U/D
        # Dof4: Wrist rotation: Axis 0 : D-Pad L/R
        # Gripper: Button 0 : Button X



        c1 = self.j1.get_axis(2)
        (cx,c2) = self.j1.get_hat(0)
        c3 = -self.j1.get_axis(1)
        c4 = self.j1.get_axis(0)
        c5 = self.j1.get_button(0)
        c6 = self.j1.get_button(1)
        # c1 = j1.get_axis(2)
        # c2 = j1.get_axis(3)
        # c3 = j1.get_axis(5)
        # c4 = j1.get_axis(0)
        # c5 = j1.get_button(0)
        

        # Assign commands for each Dof:

        cvec = [c1,c2,c3,c4,c5]
        cmd_scale = [0.2, 0.2, 0.1, 0.3, 1]

        grip_close = -0.3
        grip_open = 0.3

        for i in range(len(cvec)):
            if not i == 4:
                self.command[i] = self.currval[i] + np.sign(cvec[i])*cmd_scale[i]
            elif c5 > 0 :
                self.command[i] = self.currval[i] + grip_close
            elif c6 > 0 :
                self.command[i] = self.currval[i] + grip_open
            # elif cvec[i] == True:
            #     if math.fabs(self.currval[i] - grip_close)<0.2:
            #         self.command[i] = grip_open
            #     else:
            #         self.command[i] = grip_close
        
        #Temp code        
        

        #rospy.loginfo('Command to be published: {0}', str(self.command))                    
        print self.command
        self.publish_cmd()

    def publish_cmd(self):
        for i in range(len(self.pubvec)):
            self.pubvec[i].publish(self.command[i])

    def execute(self):
        r = rospy.Rate(10.0)
        clock = pygame.time.Clock()
        while not rospy.is_shutdown():
            clock.tick(10)
            self.motor_sub = rospy.Subscriber('/motor_states/third_arm_port', dynamixel_msgs.msg.MotorStateList, self.arm_callback)           
            r.sleep()

    def test_joystick(self):
        pygame.event.get()
        name = self.j1.get_name()
        print "Joystick name: " + name + "\n"
        
        axes = self.j1.get_numaxes()
        print "Joystick axes: {}".format(axes) 
        for i in range(axes):
            ax = self.j1.get_axis(i)
            print "Axis {0} Value {1}".format(i,ax)        

        buttons = self.j1.get_numbuttons()
        print "Joystick buttons: {}".format(buttons) 
        for i in range(buttons):
            ax = self.j1.get_button(i)
            print "Button {0} Value {1}".format(i,ax)        
        
        hats = self.j1.get_numhats()
        print "Joystick hats: {}".format(hats) 
        for i in range(hats):
            ax = self.j1.get_hat(i)
            print "Hat {0} Value {1}".format(i,ax)        


    def test(self):
        clock = pygame.time.Clock()
        #r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            clock.tick(10)
            self.get_joystick()
        


if __name__ == '__main__':
    t1 = thirdarm_joystick()
    t1.test()
    