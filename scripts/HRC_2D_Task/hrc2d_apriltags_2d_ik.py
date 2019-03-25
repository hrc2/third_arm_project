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

import random
import arbotix_msgs.srv
import dynamixel_msgs.msg
import dynamixel_controllers.srv
from diagnostic_msgs.msg import DiagnosticArray
from apriltags_ros.msg import AprilTagDetectionArray
import csv


class ik_2d_apriltags:
    # currval = [0.0, 0.0]
    # command = [0.0, 0.0]


    def __init__(self):
        rospy.init_node('apriltags_2d_poser')
        self.pub_base_pose = rospy.Publisher('/base_pose', Point, queue_size=1)
        self.pub_ee_pose = rospy.Publisher('/ee_pose', Point, queue_size=1)
        self.pub_cup1_pose = rospy.Publisher('/cup1_pose', Point, queue_size=1)
        self.pub_box1_pose = rospy.Publisher('/box1_pose', Point, queue_size=1)
        self.pub_cup2_pose = rospy.Publisher('/cup2_pose', Point, queue_size=1)
        self.pub_box2_pose = rospy.Publisher('/box2_pose', Point, queue_size=1)
        self.pubvec = [self.pub_base_pose, self.pub_ee_pose, self.pub_cup1_pose, self.pub_box1_pose, self.pub_cup2_pose, self.pub_box2_pose]
        self.tag_ids = [1, 2, 5, 8, 9, 10]


    def positions_update(self, data):
        try:
            for i in range(len(self.tag_ids)):
                ind = np.flatnonzero(np.array(self.tag_ids) == data.detections[i].id)
                self.publish_poses(ind[0], data.detections[i].pose.pose.position)


        except (NameError, IndexError):
            return



    def publish_poses(self, index, position):
        self.pubvec[index].publish(position)

        #    def pos_msg_update(self):


    def run(self):
        flag1 = 0
        flag2 = 0
        while (1 - flag1 * flag2):
            self.tags = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
            try:
                x1 = self.tags.detections[0].pose.pose.position.x
                flag1 = 1
                # break
            except (NameError, IndexError):
                continue
            try:
                x2 = self.tags.detections[1].pose.pose.position.x
                x3 = self.tags.detections[2].pose.pose.position.x
                flag2 = 1
                # break
            except (NameError, IndexError):
                continue

        print("Detected Initial States: ")


        if self.tags.detections[0].pose.pose.position.x != 0 and self.tags.detections[1].pose.pose.position.x != 0:
            self.base_x0 = self.tags.detections[0].pose.pose.position.x
            self.base_y0 = self.tags.detections[0].pose.pose.position.y
            self.ee_x0 = self.tags.detections[1].pose.pose.position.x
            self.ee_y0 = self.tags.detections[1].pose.pose.position.y

            print("Base Intials - X: " + str(self.base_x0) + " Y: " + str(self.base_y0))
            print("Gripper Intials - X: " + str(self.ee_x0) + " Y: " + str(self.ee_y0))
            time.sleep(0.2)

        # while not rospy.is_shutdown():
        self.pos_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.positions_update)
        rospy.spin()  # self.rate.sleep()






if __name__ == '__main__':
    t1 = ik_2d_apriltags()
    t1.run()
