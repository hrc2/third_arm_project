#! /usr/bin/env python3

import sys
import os

import rospy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from scripts.motion_planner_controller import third_arm_motion_planner

class WRTA_ROS_controller_interface:
    """A program to control Third Arm via PyPot"""
    def __init__(self):
        # #########################Fields#####################################

        self.loop_rate = rospy.Rate(20) # loop_rate.sleep() # 20 Hz

        self.motion_planner = third_arm_motion_planner()

        self.incomplete = True

        # ################### Subscribers ####################################

        # Optitrack subscriber

        # ################### Publishers ####################################

        # ################### Actions ######################################

        # ################### Services ####################################

    # ################### Action Clients ###################################

    # ################### Service Clients ##################################

    # ################### Callback Functions ###############################

    # callback for optitrack

    def optitrack_callback(self):
        """ get pose of human and third arm using optitrack data """

        self.motion_planner.motion_callback(new_human_pos, new_third_arm_pos)

    # ################### Methods ###########################################

    def control_loop(self):
        """ control loop for updating arm """

        while (self.incomplete):
            self.motion_planner.control_main_ROS()
            loop_rate.sleep()

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('node_name', anonymous=False)

    try:
        third_arm_brain = WRTA_ROS_controller_interface()
    except rospy.ROSInterruptException:
        pass