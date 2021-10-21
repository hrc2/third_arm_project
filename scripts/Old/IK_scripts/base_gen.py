#!/usr/bin/env python
# Generate random trajectories for the base link

import numpy as np
from numpy import *
import time
import rospy
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion
from sensor_msgs.msg import JointState
import tf
import math
import time

def generate():
	traj_pub = 
	flag = rospy.wait_for_message('/traj_flag', JointState)