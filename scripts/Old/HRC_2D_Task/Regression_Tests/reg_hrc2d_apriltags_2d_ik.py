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
        self.pub_cup_pose = rospy.Publisher('/cup_pose', Point, queue_size=1)
        self.pub_cup1_pose = rospy.Publisher('/cup1_pose', Point, queue_size=1)
        self.pub_box1_pose = rospy.Publisher('/box1_pose', Point, queue_size=1)
        self.pub_cup2_pose = rospy.Publisher('/cup2_pose', Point, queue_size=1)
        self.pub_box2_pose = rospy.Publisher('/box2_pose', Point, queue_size=1)
        self.pub_left_hand_pose = rospy.Publisher('/left_hand_pose', Point, queue_size=1)
        self.pub_right_hand_pose = rospy.Publisher('/right_hand_pose', Point, queue_size=1)
        self.pubvec = [self.pub_base_pose, self.pub_ee_pose, self.pub_cup1_pose, self.pub_box1_pose, self.pub_cup2_pose, self.pub_box2_pose, self.pub_left_hand_pose, self.pub_right_hand_pose, self.pub_cup_pose]
        self.tag_ids = [1, 2, 5, 8, 9, 10, 3, 4, 12]
        self.pos_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.positions_update)


    def positions_update(self, data):
        self.tag_data = data

    def continuous_publisher(self):
        try:
            for i in range(len(self.tag_ids)):
                ind = np.flatnonzero(np.array(self.tag_ids) == self.tag_data.detections[i].id)
                self.publish_pose(ind[0], self.tag_data.detections[i].pose.pose.position)

        except (NameError, IndexError):
            return

    def publish_pose(self, index, position):
        self.pubvec[index].publish(position)


    def run(self):
        flag1 = 0
        #flag2 = 0
        while (1 - flag1):
            #print("Checking")
            self.tag_data = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
            check_list = [1, 2, 3, 4]
            check_ctr = 0
            try:
                for i in range(len(self.tag_ids)):
                    if (self.tag_data.detections[i].id in check_list):
                        #print (self.tags.detections[i].id)
                        check_ctr += 1
                        check_list.remove(self.tag_data.detections[i].id)
                        #print(check_list)
                        #print(check_ctr)
                    if check_ctr == 4:
                        flag1 = 1
                        break
            except (NameError, IndexError):
                continue


        print("Detected Initial States: ")


        # if self.tags.detections[0].pose.pose.position.x != 0 and self.tags.detections[1].pose.pose.position.x != 0:
        #     self.base_x0 = self.tags.detections[0].pose.pose.position.x
        #     self.base_y0 = self.tags.detections[0].pose.pose.position.y
        #     self.ee_x0 = self.tags.detections[1].pose.pose.position.x
        #     self.ee_y0 = self.tags.detections[1].pose.pose.position.y
        #
        #     print("Base Intials - X: " + str(self.base_x0) + " Y: " + str(self.base_y0))
        #     print("Gripper Intials - X: " + str(self.ee_x0) + " Y: " + str(self.ee_y0))
        #     time.sleep(0.2)

        while not rospy.is_shutdown():
            self.continuous_publisher()

        rospy.spin()  # self.rate.sleep()




if __name__ == '__main__':
    t1 = ik_2d_apriltags()
    t1.run()