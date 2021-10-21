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
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64
from visualization_msgs.msg import Marker
import random
import arbotix_msgs.srv
import dynamixel_msgs.msg
import dynamixel_controllers.srv
from diagnostic_msgs.msg import DiagnosticArray
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
import csv

class csv_from_bag:    

    #currval = [0.0, 0.0]
    #command = [0.0, 0.0]


    def __init__(self):
        rospy.init_node('csv_writer')
        #self.pub_base_pose = rospy.Publisher('/base_pose', Point, queue_size=1)
        #self.pub_ee_pose = rospy.Publisher('/ee_pose', Point, queue_size=1)
        


    def skeleton_writer(self, data):
	self.skeleton_time = data.header.stamp.to_sec()
	#print("Skeleton markers timestamp: " + str(self.skeleton_time))
	points = data.points
	p_arr = []
	for point in points:
		p_arr.extend([point.x, point.y, point.z])
	p_arr.extend([self.skeleton_time])
	#print("Skeleton markers data as array: " + str(p_arr))
	#print(len(p_arr))
	with open('SkeletonMarkers.csv','a') as f:			
		writer = csv.writer(f,quoting=csv.QUOTE_ALL)
		writer.writerow(p_arr) 

    def tags_writer(self, data):
	detect  = AprilTagDetection()
	try:	
		#print(len(data.detections))
		for detect in data.detections:
			pdata = []
			frame_id = detect.id
			pos = [detect.pose.pose.position.x, detect.pose.pose.position.y, detect.pose.pose.position.z]
			pdata.extend(pos)
			pdata.extend([detect.pose.header.stamp.to_sec()])
			#print("April Tag ID: "+ str(frame_id) + " WriteData: " + str(pdata))
			with open(str('AprilTag' + str(frame_id) + '.csv'),'a') as f:			
				writer = csv.writer(f,quoting=csv.QUOTE_ALL)
				writer.writerow(pdata) 	
			
   	except (NameError, IndexError):
		return

    def publish_poses(self, base_pose, ee_pose):
	self.pub_base_pose.publish(base_pose)
	self.pub_ee_pose.publish(ee_pose)



    def run(self):

    	self.tags_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tags_writer)
	self.skeleton_sub = rospy.Subscriber('/skeleton_markers', Marker, self.skeleton_writer)
	rospy.spin()
			#self.rate.sleep()
			#self.pos_msg_update()
		    	#self.positions_display()



if __name__ == '__main__':
    t1 = csv_from_bag()
    t1.run()
