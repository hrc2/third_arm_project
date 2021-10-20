#! /usr/bin/env python
"""A program to control Third Arm via dynamixel_command service"""

from __future__ import print_function

import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import genpy

# Class to control third arm

class ThirdArmController():
    def __init__(self):
        # #########################Fields#####################################
        
        # ################### Subscribers ####################################

        # ################### Publishers ####################################
        trajectory_topic_name = '/dynamixel_workbench/joint_trajectory'
        self.trajectory_publisher = rospy.Publisher(trajectory_topic_name, JointTrajectory, queue_size=10)

        # ################### Actions ######################################

        # ################### Services ####################################
        command_service = "/dynamixel_workbench/dynamixel_command"
        self.command_service_client = rospy.ServiceProxy(command_service, DynamixelCommand)
        print('Waiting for dynamixel_command service')
        rospy.wait_for_service(command_service)
        print('dynamixel_command service connected')

        execution_service = "/dynamixel_workbench/execution"
        self.execution_service_client = rospy.ServiceProxy(execution_service, Trigger)
        print('Waiting for execution_service service')
        rospy.wait_for_service(execution_service)
        print('execution_service service connected')

        # Startup Procedure ##
        self.sendCommandClient(5, 'Moving_Speed', 100)
        # self.sendCommandClient(5, 'Goal Velocity', 0.4)
        # input('Hit enter to send trajectory')
        # self.send_joint_trajectory()
        # input('Hit enter to execute')
        # self.sendExecutionClient()
        # input()
        rospy.sleep(0.5)
        # self.sendCommandClient(5, 'Moving_Speed', 0)
        self.sendCommandClient(5, 'Goal Velocity', 0.0)
        rospy.sleep(2.0)


    # ################### Action Clients ###################################
    # def actionClient():

    # ################### Service Clients ##################################
    def sendCommandClient(self, id, addr_name, value):
        try: 
            command = ""
            #id = int(input('Motor: '))
            # addr_name = "Goal_Position"
            #value = int(input('value: '))
            response = self.command_service_client(command, id, addr_name, value)

            if response:
                print('Successful command')
            else:
                print('Failed command', str(value), 'for motor', str(id))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def sendExecutionClient(self):
        try: 
            message = []
            response = self.execution_service_client()

            if response:
                print('Successful execution')
            else:
                print('Failed execution')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    # ################### Callback Functions ###############################
    # def callback():

    # ################### Methods ###########################################
    def send_joint_trajectory(self):
        new_trajectory = JointTrajectory()
        new_trajectory.joint_names = ['gripper']
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [2300,2700]
        gripper_point.time_from_start = genpy.Duration(2)
        gripper_point.accelerations = [1]
        gripper_point.effort = [1]
        gripper_point.velocities = [1]

        #[JointTrajectoryPoint(positions= [2300, 2700], time_from_start=genpy.Duration(2))]

        self.trajectory_publisher.publish(new_trajectory)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('node_name', anonymous=False)

    try:
        third_arm_brain = ThirdArmController()
    except rospy.ROSInterruptException:
        pass






# class data:
#     command = ""
#     id = 0
#     addr_name = ""
#     value = 0



# if __name__ == "__main__":
#     while(True):
#         send_command_arm()