import time

from contextlib import closing

import pypot.robot
from pypot.dynamixel import DxlIO, motor

from PyPotMotorConfig import third_arm_robot_config, control_config

def test_velocity(robot_part):
    """ robot_part is part want to control """
    print("Starting test:", test_velocity.__name__, test_position_control.__name__, str(robot_part))
    
    print('speed -50')
    robot_part.goal_speed = -50
    time.sleep(5)
    print('speed 50')
    robot_part.goal_speed = 50
    time.sleep(5)
    robot_part.goal_speed = 0
    print("Ending test:", test_velocity.__name__)
    print()

def test_position_control(robot_part, max, min):
    """ robot_part is part want to control """
    print("Starting test:", test_position_control.__name__, str(robot_part))
    print('min pos')
    robot_part.goal_position = min
    time.sleep(4)
    print('max pos')
    robot_part.goal_position = max
    time.sleep(4)
    print("Ending test:", test_position_control.__name__)
    print()

def test_position_control_speed(robot_part, max, min):
    """ robot_part is part want to control """
    print("Starting test:", test_position_control.__name__, str(robot_part))
    print('min pos')
    robot_part.goal_position = min/2
    time.sleep(0.05)
    print('max pos')
    robot_part.goal_position = min
    time.sleep(0.05)
    robot_part.goal_position = min/2
    time.sleep(0.05)
    print('max pos')
    robot_part.goal_position = max/2
    time.sleep(0.05)
    robot_part.goal_position = max
    time.sleep(0.05)
    print('max pos')
    robot_part.goal_position = max/2
    time.sleep(0.05)
    print("Ending test:", test_position_control.__name__)
    print()

def save_speeds(robot_class):
    """ Save current speeds """
     
    motor_speeds = {}
    for motor in third_arm_robot_config['motors']:
        motor_speeds[motor] = robot_class.__getattribute__(motor).moving_speed

    return motor_speeds

def set_speeds(robot_class, speeds = {}):
    """ Set new speeds """
    
    motor_speeds = {}
    for motor in third_arm_robot_config['motors']:
        if speeds == {}:
            speed = third_arm_robot_config['motors'][motor]['moving_speed']
        else:
            speed = speeds[motor]
        robot_class.__getattribute__(motor).moving_speed = speed
        print(motor, 'is now at', str(speed))

# The closing decorator make sure that the close function will be called
# on the object passed as argument when the with block is exited.
with closing(pypot.robot.from_config(third_arm_robot_config)) as my_robot:
    # do stuff without having to make sure not to forget to close my_robot!

    # save the current values of the speed
    old_speeds = save_speeds(my_robot)
    set_speeds(my_robot)

    try:

        # need to run base DXL control to first control the motor
        # otherwise will not work
        for motor in my_robot.motors:
            motor.compliant = False
        test_position_control_speed(my_robot.wrist_axiel, control_config['wrist_axiel']['max'], control_config['wrist_axiel']['min'])
        # # gripper
        # test_position_control(my_robot.gripper, control_config['gripper']['max'], control_config['gripper']['min'])
        # test_velocity(my_robot.gripper)
        # # wrist axiel
        # test_position_control(my_robot.wrist_axiel, control_config['wrist_axiel']['max'], control_config['wrist_axiel']['min'])
        # test_velocity(my_robot.wrist_axiel)
        # # wrist tilt
        # test_position_control(my_robot.wrist_tilt, control_config['wrist_tilt']['max'], control_config['wrist_tilt']['min'])
        # test_velocity(my_robot.wrist_tilt)

    finally:
        # clean up and set old speeds back
        set_speeds(my_robot, old_speeds)