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

import os

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

if __name__ == "__main__":
    torso = ViconTrackerPoseHandler(None, None, "",51039, "torso")
    upper_arm = ViconTrackerPoseHandler(None, None, "", 51040, "upper_arm")
    lower_arm = ViconTrackerPoseHandler(None, None, "", 51041, "upper_arm")
    #print sys.float_info.epsilon
    #print a.getPose()
    #time.sleep(10)
    ftm = time.mktime(time.localtime())
    fname = 'Vicon' + str(ftm) + '.csv'
    while 1:

    	t_pose = torso.getPose()
    	u_pose = upper_arm.getPose()
    	l_pose = lower_arm.getPose()
    	data = []
    	data.extend(t_pose)
    	data.extend(u_pose)
    	data.extend(l_pose)
    	print data
    	
    	with open(fname,'a') as f:
    		writer = csv.writer(f)
    		writer.writerow(data)
    	time.sleep(0.02)

