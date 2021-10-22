#! /usr/bin/env python
"""A program to control Third Arm via dynamixel_command service"""

from __future__ import print_function

import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import genpy

# from MAIN_CONFIG_CLASSES import WRTA_ROS_config

# import motor_config

class WRTA_ROS_config:

    def __init__(self):
        ## TOPICS ##
        self.trajectory_topic_name = '/dynamixel_workbench/joint_trajectory'

        ## SERVICES ##

        self.command_service = "/dynamixel_workbench/dynamixel_command"
        self.execution_service = "/dynamixel_workbench/execution"

class wrist_axiel:
    motor_id = 4
    max = 1000
    min = 0
    range = abs(max-min)
    # 500 - rotate right
    # level, upright- 0
    # max - 1000, level upright

class wrist_tilt:
    motor_id = 5
    max = 1000
    min = 0
    range = abs(max-min)
    level = 500

class gripper:
    motor_id = 6
    max = 2700
    min = 2000
    range = abs(max-min)
    open = 2000
    closed = 2700


# Class to control third arm

class WRTA_ROS_controller_interface:
    def __init__(self):
        # #########################Fields#####################################

        self.config = WRTA_ROS_config()
        self.command_service_client = None
        self.execution_service_client = None


        ## temp, will be replaced with main config classes with built in support for motors
        self.wrist_axiel = wrist_axiel()
        self.wrist_tilt = wrist_tilt()
        self.gripper = gripper()
        self.goal_position = 'Goal_Position'
        self.goal_velocity = 'Goal_Velocity'
        self.moving_speed = 'Moving_Speed'


        # ################### Subscribers ####################################

        # ################### Publishers ####################################

        self.trajectory_publisher = rospy.Publisher(self.config.trajectory_topic_name, JointTrajectory, queue_size=10)

        # ################### Actions ######################################

        # ################### Services ####################################

        self.command_service_client = self.connect_to_service(self.config.command_service, DynamixelCommand)
        self.execution_service_client = self.connect_to_service(self.config.execution_service, Trigger)

        # Startup Procedure ##
        self.test_wrist_and_gripper_at_same_time_service_goal_position()


    # ################### Action Clients ###################################
    # def actionClient():

    def test_wrist_and_gripper_at_same_time_service_goal_position(self):
        """move wrist and gripper at same time using goal position command"""

        self.sendCommandClient(self.gripper.motor_id, self.goal_position, self.gripper.max)
        self.sendCommandClient(self.wrist_tilt.motor_id, self.goal_position, self.wrist_tilt.max)
        self.sendCommandClient(self.wrist_axiel.motor_id, self.goal_position, self.wrist_axiel.max)
        rospy.sleep(2.0)
        self.sendCommandClient(self.gripper.motor_id, self.goal_position, self.gripper.min)
        self.sendCommandClient(self.wrist_tilt.motor_id, self.goal_position, self.wrist_tilt.min)
        self.sendCommandClient(self.wrist_axiel.motor_id, self.goal_position, self.wrist_axiel.min)
        rospy.sleep(2.0)

    def test_wrist_and_gripper_at_same_time_service_gaol_velocity(self):
        """move wrist and gripper at same time using goal_velocity command"""

        self.sendCommandClient(self.gripper.motor_id, self.goal_velocity, .4)
        self.sendCommandClient(self.wrist_axiel.motor_id, self.goal_velocity, 1)
        rospy.sleep(.4)
        self.sendCommandClient(self.gripper.motor_id, self.goal_velocity, -.4)
        self.sendCommandClient(self.wrist_axiel.motor_id, self.goal_velocity, -1)
        rospy.sleep(.4)
        self.sendCommandClient(self.gripper.motor_id, self.goal_velocity, 0)
        self.sendCommandClient(self.wrist_axiel.motor_id, self.goal_velocity, 0)
        rospy.sleep(2.0)

    def test_joint_trajectory(self):
        """move gripper using joint trajectory"""
        input('Hit enter to send trajectory')
        self.send_joint_trajectory()
        input('Hit enter to execute')
        self.sendExecutionClient()
        input()

    # ################### Service Clients ##################################
    def sendCommandClient(self, motor_id, addr_name, value):
        try:
            command = ""
            response = self.command_service_client(command, motor_id, addr_name, value)

            if response:
                print('Successful command')
            else:
                print('Failed command', str(value), 'for motor', str(motor_id))

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def sendExecutionClient(self):
        try:
            response = self.execution_service_client()

            if response:
                print('Successful execution')
            else:
                print('Failed execution')
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    # ################### Callback Functions ###############################
    # def callback():

    # ################### Methods ###########################################
    def send_joint_trajectory(self):
        new_trajectory = JointTrajectory()
        new_trajectory.joint_names = ['gripper']
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [2300, 2700]
        gripper_point.time_from_start = genpy.Duration(2)
        gripper_point.accelerations = [1]
        gripper_point.effort = [1]
        gripper_point.velocities = [1]

        # [JointTrajectoryPoint(positions= [2300, 2700], time_from_start=genpy.Duration(2))]

        self.trajectory_publisher.publish(new_trajectory)

    def connect_to_service(self, service_name, service_root):
        """ Connect to a service

        Args:
            service_name: name of service to connect to
            service_root: where service is based from
        Returns:
            service client proxy

        """
        new_client = rospy.ServiceProxy(service_name, service_root)
        print('Waiting for', service_name)
        rospy.wait_for_service(service_name)
        print(service_name, 'connected')

        return new_client


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('node_name', anonymous=False)

    try:
        third_arm_brain = WRTA_ROS_controller_interface()
    except rospy.ROSInterruptException:
        pass