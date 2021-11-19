#! /usr/bin/env python

from scipy.spatial.transform import Rotation as R
import numpy as np

import sys
import os
import tf

import rospy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from scripts.motion_planner_controller import third_arm_motion_planner
from scripts.ROS_Config import ROS_config

class WRTA_ROS_controller_interface:
    """A program to control Third Arm via PyPot"""
    def __init__(self):
        # #########################Fields#####################################

        self.loop_rate = rospy.Rate(20) # loop_rate.sleep() # 20 Hz

        self.test_ik_rate = rospy.Rate(1)

        self.motion_planner = third_arm_motion_planner()

        self.incomplete = True

        self.config = ROS_config()

        self.transformation_base = None
        self.transformation_gripper = None
        self.transformation_other_hand = None

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

        try:
            # update positions
            base_translation, base_rotation = self.get_from_tf(self.config.opti_track_origin, self.config.third_arm_base)
            self.transformation_base = self.get_transform_from_translation_and_rotation(base_translation, base_rotation)

            gripper_translation, gripper_rotation = self.get_from_tf(self.config.opti_track_origin, self.config.third_arm_gripper)
            self.transformation_gripper = self.get_transform_from_translation_and_rotation(base_translation, base_rotation)

            other_hand_translation, other_hand_rotation = self.get_from_tf(self.config.opti_track_origin, self.config.third_arm_other_hand)
            self.transformation_other_hand = self.get_transform_from_translation_and_rotation(other_hand_translation, other_hand_rotation)

            # transofrmation matrix with numpy for posion and quaterinon  vector
            # send aove magrix to ik solver in same loop and then get joint agnels then command said jintnangles

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('Could not find transform')

        if self.transformation_base is not None and self.transformation_gripper is not None:
            self.compareToIK()

        # self.motion_planner.motion_callback(new_human_pos, new_third_arm_pos)

    # ################### Methods ###########################################

    def get_transform_from_translation_and_rotation(translation, rotation):
        """ get the transformation matrix from the tranlsation and rotation arrays """

        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html
        # http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
        # https://stackoverflow.com/questions/40833073/insert-matrix-into-the-center-of-another-matrix-in-python

        rotation_matrix = R.from_quat(rotation)
        rotation_matrix = rotation_matrix.as_matrix()
        # assemble transform

        transform = np.zeros((4, 4))

        # add rotation
        transform[0:3, 0:3] = rotation_matrix
        
        # add transform
        translation = np.array(translation)
        transform[0:3, 3:4] = translation.reshape((3, 1))

        transform[3][3] = 1

        return transform

    def compareToIK(self):
        """ test current position to IK solver """

        gripper_to_base_transform = np.matmul(np.linalg.inv(self.transformation_base), self.transformation_gripper)
        success, thetas = self.motion_planner.IKSolver.solve_kinematics(gripper_to_base_transform)
        print()
        print()
        print()
        print("IK Successful:", str(success))
        print("IK Output:", str(thetas.tolist()))
        print("Current Robot thetas:", str(self.motion_planner.get_angles()))
 
        self.loop_rate.sleep()

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('node_name', anonymous=False)

    try:
        third_arm_brain = WRTA_ROS_controller_interface()
        third_arm_brain.control_loop()
    except rospy.ROSInterruptException:
        pass