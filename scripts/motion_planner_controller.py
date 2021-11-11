import time

from contextlib import closing
import pypot.robot
from pypot.dynamixel import DxlIO, motor

from PyPotMotorConfig import third_arm_robot_config, control_config, motors_types

from PyPotController import control_motor, control_payload
import motion_planner_config



class third_arm_motion_planner:
    """ The class for controlling the third arm"""

    def __init__(self):
        self.config = motion_planner_config.motion_planning_controller_config()

        self.human_position = [] # Human hand's position received from optitrack
        self.third_arm_positions = []

        self.control_payload = control_payload()

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
        
        self.plan_with_kinematics()

        # move motors

        for motor in self.motor_controllers:
            if self.control_payload.motor.pos == True:
                self.motor_controllers[motor].move(self.control_payload.motor.command)
            else:
                self.motor_controllers[motor].move_with_speed(self.control_payload.motor.command)

   
    def plan_with_kinematics(self):
        """ updates control_payload with new values from inverse kinematics """

        