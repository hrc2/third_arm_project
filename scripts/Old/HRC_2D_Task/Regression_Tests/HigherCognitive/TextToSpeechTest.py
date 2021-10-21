#!/usr/bin/env python
import rospy
import os
from playsound import playsound

#spath = '/home/hriclass/catkin_ws/src/third_arm/scripts/HRC_2D_Task/Regression_Tests/HigherCognitive/'
spath = os.path.dirname(__file__)
sfile = str(spath + '/going.wav')
rospy.init_node('sound_speaker')
playsound(sfile)
