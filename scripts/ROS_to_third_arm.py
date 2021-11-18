#! /usr/bin/env python

import sys
import os
import tf

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

        self.tf_listener = tf.TransformListener() #Optitrack tf listner
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

        while self.incomplete and not rospy.is_shutdown():
            try:
                (trans_third_arm_gripper, rot_third_arm_gripper) = self.tf_listener.lookupTransform("optitrack_origin", "third_arm_other_hand", rospy.Time(0))
                # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
                # (trans_human, rot_human) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_human_hand, rospy.Time(0))
                # print("Human position =", trans_human)
                print("Robot position =", trans_third_arm_gripper)
                print("Robot orientation =", rot_third_arm_gripper)
                print(self.motion_planner.get_angles())
                # transofrmation matrix with numpy for posion and quaterinon  vector
                # send aove magrix to ik solver in same loop and then get joint agnels then command said jintnangles
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'Could not find transform'
                continue
            # self.motion_planner.control_main_ROS()
            self.loop_rate.sleep()

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('node_name', anonymous=False)

    try:
        third_arm_brain = WRTA_ROS_controller_interface()
        third_arm_brain.control_loop()
    except rospy.ROSInterruptException:
        pass