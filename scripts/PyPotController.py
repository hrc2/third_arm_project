import time

from contextlib import closing

import pypot.robot
from pypot.dynamixel import DxlIO

import PyPotMotorConfig

# The closing decorator make sure that the close function will be called
# on the object passed as argument when the with block is exited.


with closing(pypot.robot.from_config(PyPotMotorConfig.third_arm_robot_config)) as my_robot:
    # do stuff without having to make sure not to forget to close my_robot!

    #test velocity/wheel mode
    print('start test 1')
    print(my_robot.wrist_axiel.present_position)
    my_robot.wrist_axiel.goal_speed = 10
    time.sleep(3)
    my_robot.wrist_axiel.goal_speed = -10
    time.sleep(3)
    my_robot.wrist_axiel.goal_speed = 0

    # test position control mode
    print('start test 2')
    my_robot.wrist_axiel.goal_position = 90
    time.sleep(3)
    my_robot.wrist_axiel.goal_position = -100

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

    def move_with_speed(self, speed):
        """ move the motor with a speed """

        self.robot.__getattribute__(self.motor_name).goal_speed = speed

