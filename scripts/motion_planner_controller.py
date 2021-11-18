import time

from contextlib import closing
import pypot.robot
from pypot.dynamixel import DxlIO, motor

import sys
import os

# ROS import way
try:
    from scripts.PyPotMotorConfig import third_arm_robot_config, control_config, motors_types
    from scripts.PyPotController import control_motor, control_payload
    from scripts.motion_planner_config import motion_planning_controller_config
    from scripts.inverse_kinematics import InverseKinematicsSolver

# regular import way
except:
    from PyPotMotorConfig import third_arm_robot_config, control_config, motors_types
    from PyPotController import control_motor, control_payload
    from motion_planner_config import motion_planning_controller_config
    from inverse_kinematics import InverseKinematicsSolver

class third_arm_motion_planner:
    """ The class for controlling the third arm"""

    def __init__(self):
        self.config = motion_planning_controller_config()

        self.human_position = [] # Human hand's position received from optitrack
        self.third_arm_positions = []

        self.control_payload = control_payload()

        self.IKSolver = InverseKinematicsSolver()

        # load robot config
        self.robot = pypot.robot.from_config(third_arm_robot_config)

        # initialze the motors, reuqired for robot controls to work
        self.init_motors()

        # set the motor speeds
        self.set_initial_motor_speed()

        # make motor controllers
        self.motor_controllers = {}
        for motor in third_arm_robot_config['motors']:
            self.motor_controllers[motor] = control_motor(motor, third_arm_robot_config['motors'][motor]['moving_speed'], self.robot)

    def init_motors(self):
        """ initilize the motors by setting compliance to false """
        
        for motor in self.robot.motors:
            motor.compliant = False

    def set_initial_motor_speed(self):

        """ initilize the motor speeds """
        
        for motor in third_arm_robot_config['motors']:

            # get speed from config
            speed = third_arm_robot_config['motors'][motor]['moving_speed']

            # set speed
            self.robot.__getattribute__(motor).moving_speed = speed
            print(motor, 'is now at', str(speed))

    def motion_callback(self, new_human_pos, new_third_arm_pos):
        """ callback to be called by ROS for using Optitrack data """

        self.human_position = new_human_pos
        self.third_arm_positions = new_human_pos

    def control_main_ROS(self):

        # plan from human and arm position
        
        new_output_joints = self.plan_with_kinematics()

        # do motion planning if possible with new_output_joints

        self.motion_planning(new_output_joints)

        # move motors

        self.move_with_payloads()

    def get_angles(self):
        """ returns angles of all motors """

        angles = {}
        for motor in self.motor_controllers:
            angles[motor] = self.motor_controllers[motor].get_position()

        return angles
        
    def plan_with_kinematics(self, input_matrix):
        """ updates control_payload with new values from inverse kinematics
        
        input_matrix is a 4x4 homogeneous transformation matrix T
        
         """
        
        valid, output_joints  = self.IKSolver.solve_kinematics(input_matrix)

        new_output_joints = {}

        if valid:
            new_output_joints['base_swivel'] = output_joints[0]
            new_output_joints['vertical_tilt'] = output_joints[1]
            new_output_joints['arm_extension'] = output_joints[2]
            new_output_joints['wrist_axiel'] = output_joints[3]
            new_output_joints['wrist_tilt'] = output_joints[4]
            new_output_joints['gripper'] = control_gripper_with_distance()

        return new_output_joints
            # for motor in third_arm_robot_config['motors']:
            #     self.control_payload.__getattribute__(motor).command = new_output_joints[motor]
            # if was doing no motion planning and just using new_output_joints

    def control_gripper_with_distance(self, ):
        """ Control the gripper by checking how close it is to the human hand"""

        # TODO

        return 0

    def motion_planning(self, joint_thetas):
        """ do the motion planning for the third arm """

        # TODO

    def move_with_payloads(self, ):
        """ move the motors using the payload in control_payload """

        for motor in self.motor_controllers:
            if self.control_payload.motor.pos == True:
                self.motor_controllers[motor].move(self.control_payload.motor.command)
            else:
                self.motor_controllers[motor].move_with_speed(self.control_payload.motor.command)