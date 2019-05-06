#!/usr/bin/env python

import rospy

import math
import time
import numpy as np

import sensor_msgs.msg
from sensor_msgs.msg import Imu, JointState, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64, Int32, String, Float32MultiArray

class hrc2d_prob_plotter:
    def __init__(self):
        print('Starting probability parser')
        rospy.init_node('prob_plot_data_prep')
        self.prob_topic = '/target_probs'
        self.prob_init = rospy.wait_for_message(str(self.prob_topic), Float32MultiArray)
        self.get_initial()

        self.pub_topics = []
        self.pubvec = []
        for i in range(len(self.dat)):
            self.pub_topics.append(str('/target_prob_' + str(i)))
            print(self.pub_topics[i])
            self.pubvec.append(rospy.Publisher(str(self.pub_topics[i]), Float64, queue_size=1))

    def get_initial(self):
        self.prob_init = rospy.wait_for_message('/target_probs', Float32MultiArray)
        print('Received')
        self.dat = self.prob_init.data
        print(self.prob_init)

    def update_probs(self, cb):
        print(cb.data)
        for i in range(len(cb.data)):
           self.pubvec[i].publish(cb.data[i])

    def run(self):
        #rospy.Subscriber(self.prob_topic, self.update_probs)
        rospy.Subscriber('/target_probs', Float32MultiArray, self.update_probs)
        rospy.spin()


if __name__ == '__main__':
    t1 = hrc2d_prob_plotter()
    t1.run()