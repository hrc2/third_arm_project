#! /usr/bin/env python

from scipy.spatial.transform import Rotation as R
import numpy as np

import sys
import os
import tf

import rospy

#TODO add offset for base to gripper since center not exact


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
        self.transformation_gripper_base = None
        self.transformation_otherHand_base = None

        # ################### Subscribers ####################################

        self.tf_listener = tf.TransformListener() #Optitrack tf listner
        # ################### Publishers ####################################

        # ################### Actions ######################################

        # ################### Services ####################################

    # ################### Action Clients ###################################

    # ################### Service Clients ##################################

    # ################### Callback Functions ###############################

    # callback for optitrack
    def control_loop(self):
        """ control loop for updating arm """

        while self.incomplete and not rospy.is_shutdown():
            try:
                self.optitrack_callback()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print ('Could not find transform')
                continue
            if self.transformation_otherHand_base is not None: 
                self.motion_planner.control_main_ROS(self.transformation_otherHand_base)
            self.loop_rate.sleep()

    def get_from_tf(self,origin, frame):
        return self.tf_listener.lookupTransform(origin, frame, rospy.Time(0))

    def optitrack_callback(self):
        """ get pose of human and third arm using optitrack data """

        try:
            # update positions
            base_translation, base_rotation = self.get_from_tf(self.config.opti_track_origin, self.config.third_arm_base)
            self.transformation_base = self.get_transform_from_translation_and_rotation(base_translation, base_rotation)

            gripper_translation, gripper_rotation = self.get_from_tf(self.config.opti_track_origin, self.config.third_arm_gripper)
            self.transformation_gripper = self.get_transform_from_translation_and_rotation(gripper_translation, gripper_rotation)

            other_hand_translation, other_hand_rotation = self.get_from_tf(self.config.opti_track_origin, self.config.third_arm_other_hand)
            self.transformation_other_hand = self.get_transform_from_translation_and_rotation(other_hand_translation, other_hand_rotation)

            # transofrmation matrix with numpy for posion and quaterinon  vector
            # send aove magrix to ik solver in same loop and then get joint agnels then command said jintnangles
            
            gripper_translation_base, gripper_rotation_base = self.get_from_tf(self.config.third_arm_base, self.config.third_arm_gripper)
            self.transformation_gripper_base = self.get_transform_from_translation_and_rotation(gripper_translation_base, gripper_rotation_base)
            self.transformation_gripper_base += self.config.base_offset_for_IK

            otherHand_translation_base, otherHand_rotation_base = self.get_from_tf(self.config.third_arm_base, self.config.third_arm_other_hand)
            self.transformation_otherHand_base = self.get_transform_from_translation_and_rotation(otherHand_translation_base, otherHand_rotation_base)
            self.transformation_otherHand_base += self.config.base_offset_for_IK

            # print(base_translation)
            # print(gripper_translation)
            print("Gripper translation:", str(gripper_translation_base))
            print("Gripper rotation:", str(gripper_rotation_base))
            
            
            print("Current Robot thetas:", str(self.motion_planner.get_angles()))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('Could not find transform')

        if self.transformation_gripper_base is not None and self.transformation_gripper is not None:
            self.compareToIK()

        # self.motion_planner.motion_callback(new_human_pos, new_third_arm_pos)

    # ################### Methods ###########################################

    def get_transform_from_translation_and_rotation(self, translation, rotation):
        """ get the transformation matrix from the tranlsation and rotation arrays """

        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html
        # http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
        # https://stackoverflow.com/questions/40833073/insert-matrix-into-the-center-of-another-matrix-in-python

        rotation_matrix = R.from_quat(rotation)
        rotation_matrix = rotation_matrix.as_dcm()
        # rotation_matrix = rotation_matrix.as_matrix()
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

        # gripper_to_base_transform = np.linalg.inv(self.transformation_base)* self.transformation_gripper
        otherHand_to_base_transform = self.transformation_otherHand_base
        print(otherHand_to_base_transform)
        success, thetas = self.motion_planner.IKSolver.solve_kinematics(otherHand_to_base_transform)
        thetas[0] = -thetas[0]
        print()
        print()
        print()
        print("IK Successful:", str(success))
        print("IK Output:", str(thetas.tolist()))
        current_angles = self.motion_planner.get_angles()
        print("Current Robot thetas:", str(current_angles["base_swivel"]), str(current_angles["vertical_tilt"]), str(current_angles["arm_extension"]),str(current_angles["wrist_axiel"]),str(current_angles["wrist_tilt"]),str(current_angles["gripper"]))
 
        self.loop_rate.sleep()

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('node_name', anonymous=False)

    try:
        third_arm_brain = WRTA_ROS_controller_interface()
        third_arm_brain.control_loop()
    except rospy.ROSInterruptException:
        third_arm_brain.motion_planner.motor_controllers["base_swivel"].move_with_speed[0.0]
        pass