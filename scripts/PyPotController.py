import time
import math
from contextlib import closing

import pypot.robot
from pypot.dynamixel import DxlIO, motor

import PyPotMotorConfig

class motor_payload:
    """ class used to hold one motor's commands """
    
    def __init__(self):
        self.pos = True
        self.commmand = 0

    def set_payload(self, pos, command):
        """ Set the payload for this motor 
        
        Args:
            pos: true for position control, false for velocity control
            command: value to set for position or velocity control
        """
        self.pos = pos
        self.commmand = command
    

class control_payload:
    """ class used to control all motors from an input """

    def __init__(self):
        self.gripper = motor_payload()
        self.wrist_tilt = motor_payload()
        self.wrist_axial = motor_payload()
        self.base_swivel = motor_payload()
        self.vertical_tilt = motor_payload()
        self.arm_extension = motor_payload()
# check IK using current angles then plugging in Base to gripper H to check if IK is correct

# class used to control motors and not have to call functions directly and have checks in place
class control_motor:
    def __init__(self, motor_name, default_speed, robot):
        self.motor_name = motor_name
        self.default_speed = default_speed
        self.robot = robot
        self.config = PyPotMotorConfig.control_config[motor_name]

    def move(self, position, speed = None):
        """ move the motor to a position with a certain or default speed i no speed specified"""

        # check bounds
        if position > self.config['max']:
            position = self.config['max']

        if position < self.config['min']:
            position = self.config['min']

        # make sure at default or given speed
        if speed is None:
            self.set_speed(self.default_speed)
        else:
            self.set_speed(speed)

        # move motor
        self.robot.__getattribute__(self.motor_name).goal_position = position

    def move_rad(self, position, speed = None):
        """ move the motor to a RADIAN position with a certain or default speed i no speed specified"""
        self.move(position*180/math.pi, speed)

    def get_position(self):
        """ return motor's position"""
        return self.robot.__getattribute__(self.motor_name).present_position*math.pi/180

    def set_speed(self, speed):
        """ set the speed of the motor """
        self.robot.__getattribute__(self.motor_name).moving_speed = speed

    def move_with_speed(self, speed):
        """ move the motor with a speed """
        self.robot.__getattribute__(self.motor_name).goal_speed = speed

    def move_with_rad_speed(self, speed):
        """ move the motor with a speed """
        self.robot.__getattribute__(self.motor_name).goal_speed = speed*180/math.pi
