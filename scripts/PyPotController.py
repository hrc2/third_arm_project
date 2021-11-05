import time

from contextlib import closing

import pypot.robot
from pypot.dynamixel import DxlIO

import PyPotMotorConfig


class motor_payload:
    """ class used to control all motors from an input """



# class used to control motors and not have to call functions directly and have checks in place
class control_motor:
    def __init__(self, motor_name, config, robot):
        self.motor_name = motor_name
        self.config = config
        self.robot = robot

    def move(self, position):
        """ move the motor to a position"""

        # check bounds
        if position > self.config['max']:
            position = self.config['max']

        if position < self.config['min']:
            position = self.config['min']

        # move motor
        self.robot.__getattribute__(self.motor_name).goal_position = position

    def get_position(self):
        """ return motor's position"""
        return self.robot.__getattribute__(self.motor_name).present_position

    def set_speed(self, speed):
        """ set the speed of the motor """
        self.robot.__getattribute__(self.motor_name).moving_speed = speed

    def move_with_speed(self, speed):
        """ move the motor with a speed """
        self.robot.__getattribute__(self.motor_name).goal_speed = speed

