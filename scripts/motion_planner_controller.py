import time

from contextlib import closing
import pypot.robot
from pypot.dynamixel import DxlIO, motor

import sys
import os
import numpy as np

# ROS import way
try:
    from scripts.PyPotMotorConfig import third_arm_robot_config, control_config, motors_types
    from scripts.PyPotController import control_motor, control_payload
    from scripts.motion_planner_config import motion_planning_controller_config
    from scripts.inverse_kinematics import InverseKinematicsSolver
    from scripts.toolbox import PID_template

# regular import way
except:
    from PyPotMotorConfig import third_arm_robot_config, control_config, motors_types
    from PyPotController import control_motor, control_payload
    from motion_planner_config import motion_planning_controller_config
    from inverse_kinematics import InverseKinematicsSolver
    from toolbox import PID_template

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

        # initialze the motors, required for robot controls to work
        self.init_motors()

        # set the motor speeds
        self.set_initial_motor_speed()

        # make motor controllers
        self.motor_controllers = {}
        for motor in third_arm_robot_config['motors']:
            self.motor_controllers[motor] = control_motor(motor, third_arm_robot_config['motors'][motor]['moving_speed'], self.robot)

        # PID controllers
        self.base_PID = PID_template(self.config.kp_base_swivel, 0, 0, -self.config.max_velocity, self.config.max_velocity)

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

    def control_main_ROS(self, input_matrix):

        # plan from human and arm position
        
        new_velocities, new_positions = self.plan_with_kinematics(input_matrix)

        # Joint Position Control
        # self.motor_controllers["base_swivel"].move_with_rad(new_positions["base_swivel"])

        # Joint Velocity Control
        self.motor_controllers["base_swivel"].move_with_rad_speed(new_velocities["base_swivel"])


        # move motors

        # self.move_with_payloads()

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
        
        valid, output_joints = self.IKSolver.solve_kinematics(input_matrix)
        current_joints = self.get_angles()

        new_velocities = {}
        new_positions = {}

        # get positions
        new_positions['base_swivel'] = -1.0*output_joints[0]
        # new_positions['vertical_tilt'] = output_joints[1]
        # new_positions['arm_extension'] = output_joints[2]
        # new_positions['wrist_axiel'] = output_joints[3]
        # new_positions['wrist_tilt'] = output_joints[4]
        # new_positions['gripper'] = self.control_gripper_with_distance()

        # get velocities
        new_velocities['base_swivel'] = self.base_PID.update(new_positions["base_swivel"], current_joints["base_swivel"])
        # new_velocities['vertical_tilt'] = output_joints[1]
        # new_velocities['arm_extension'] = output_joints[2]
        # new_velocities['wrist_axiel'] = output_joints[3]
        # new_velocities['wrist_tilt'] = output_joints[4]
        # new_velocities['gripper'] = self.control_gripper_with_distance()

        # set to 0 if needed
        for motor in new_velocities:
            if abs(new_velocities[motor]) < self.config.pid_velocity_threshold:
                new_velocities[motor] = 0.0

        return new_velocities, new_positions

            # for motor in third_arm_robot_config['motors']:
            #     self.control_payload.__getattribute__(motor).command = new_output_joints[motor]
            # if was doing no motion planning and just using new_output_joints

    def control_gripper_with_distance(self, ):
        """ Control the gripper by checking how close it is to the human hand"""

        # TODO

        return 0

    def move_with_payloads(self, ):
        """ move the motors using the payload in control_payload """

        for motor in self.motor_controllers:
            if self.control_payload.motor.pos == True:
                self.motor_controllers[motor].move_rad(self.control_payload.motor.command)
            else:
                self.motor_controllers[motor].move_with_rad_speed(self.control_payload.motor.command)