#!/usr/bin/env python

# ik_node testing node

import numpy as np
from numpy import *
import rospy
from geometry_msgs.msg import Pose

pub = rospy.Publisher('ik_cmd', Pose, queue_size=10)

def control():

	# Intialize the node and name it
    rospy.init_node('ik_test', anonymous=True)

	newPose = Pose()

	newPose.position.x = 0
	newPose.position.y = 0.54
	newPose.position.z = -0.25

	newPose.orientation.x = 0
	newPose.orientation.y = 0
	newPose.orientation.z = 0
	newPose.orientation.w = 1

	pub.publish(newPose)

	rospy.spin()



if __name__ == '__main__':
    # Go to the main loop
    try:
        control()
    except rospy.ROSInterruptException:
        pass