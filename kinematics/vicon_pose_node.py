#!/usr/bin/env python
"""
============================================
viconPose.py - Pose Handler for Vicon System
============================================
"""

import sys, time
from numpy import *
import socket
import struct
import threading
import csv
import rospy
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion

import os

pub = rospy.Publisher('EE_pose', Pose, queue_size=1)

class ViconTrackerPoseHandler(object):
    def __init__(self, executor, shared_data,host,port,VICON_name):
        """
        Pose handler for VICON system

        host (string): The ip address of VICON system (default="")
        port (int): The port of VICON system (default=51001)
        VICON_name (string): The name of the robot in VICON system (default="SubjectName")
        """
        self.host = host
        self.port = port
        self.VICON_name = VICON_name

        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print "Connecting..."
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.host, self.port))

        self.x = 0
        self.y = 0
        self.z = 0
        self.un = 0 #Name of object
        self.ox = 0
        self.oy = 0
        self.oz = 0
        self.t = 0
        self.thread = threading.Thread(target=self.updatePose)
        self.thread.daemon = True
        self.thread.start()
        self.t_init = time.time()

    def updatePose(self):
        while True:
            data, addr = self.s.recvfrom(256)
            body = struct.unpack("<d24s28d", data)
            #print "{} - Pose ({},{},{})".format(body, body[2],body[3],body[4])
            #print "Angle ({},{},{})\n".format(body[5]/3.14*180,body[6]/3.14*180,body[7]/3.14*180)

            # body = [timestamp?, vicon_name, tx, ty, tz, ax, ay, az, ? ....]
            # angle in radians, pose in mm            
            (self.t, self.un, self.x, self.y, self.z, self.ox, self.oy, self.oz) = [body[0], body[1], body[2]/1000, body[3]/1000, body[4]/1000, body[5], body[6] ,body[7]]

    def _stop(self):
        print "Vicon pose handler quitting..."
        self.thread.join()
        print "Terminated."

    def getPose(self, cached=False):
        #print "({t},{x},{y},{o})".format(t=t,x=x,y=y,o=o)
        self.t = time.time() #- self.t_init
        return array([self.x, self.y, self.z, self.ox, self.oy, self.oz, self.un, self.t])



def control():
    # main loop
    rospy.init_node('vicon_pose_node', anonymous=True)

    endEffector = ViconTrackerPoseHandler(None, None, "", 51042, "EE")

    while  not rospy.is_shutdown():
        EE_data = endEffector.getPose()

        if not EE_data.all():
            print "no data"
        else:
            # convert euler to quaternion
            q = Quaternion(axis=(EE_data[3], EE_data[4], EE_data[5]))

            EE_pose = Pose()
            EE_pose.position.x = EE_data[0]
            EE_pose.position.y = EE_data[1]
            EE_pose.position.z = EE_data[2]

            EE_pose.orientation.w = q.q[0]
            EE_pose.orientation.x = q.q[1]
            EE_pose.orientation.y = q.q[2]
            EE_pose.orientation.z = q.q[3]

            pub.publish(EE_pose)
            time.sleep(0.02)


if __name__ == "__main__":
    try:
        control()
    except rospy.ROSInterruptException:
        pass
