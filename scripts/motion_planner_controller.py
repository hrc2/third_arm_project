import time

from contextlib import closing
import pypot.robot
from pypot.dynamixel import DxlIO, motor

from PyPotMotorConfig import third_arm_robot_config, control_config, motors_types

from PyPotController import control_motor
import motion_planner_config



class third_arm_motion_planner:
    """ The class for controlling the third arm"""

    def __init__(self):
        self.config = motion_planner_config.motion_planning_controller_config()

        self.human_position = [] # Human hand's position received from optitrack
        self.third_arm_positions = []

        # initialze the motors, reuqired for robot controls to work
        self.init_motors()

        # load robot config
        self.robot = pypot.robot.from_config(third_arm_robot_config)

        # set the motor speeds
        self.set_initial_motor_speed()

        # make motor controllers
        self.motor_controllers = {}
        for motor in third_arm_robot_config['motors']:
            self.motor_controllers[motor] = control_motor(motor, None, self.robot)

    def init_motors(self):
        """ initilize the motors by twitching them slightly """
        dxl_io = DxlIO(self.config.port, baudrate=self.config.baudrate)
        
        # TODO check order of motors positions returned
        motor_positions = dxl_io.get_present_position(list(motors_types.keys()))
        
        for motor in motors_types:
            # now do a tiny movement, motor-1 used since motor_positions index corresponds to motor-1 when got present positions
            dxl_io.set_goal_position({motor: motor_positions[motor-1]-1})
            time.sleep(.005)
            dxl_io.set_goal_position({motor: motor_positions[motor-1]+1})
            time.sleep(.005)

            # back to where started
            dxl_io.set_goal_position({motor: motor_positions[motor-1]})
            time.sleep(.005)
        
        # close so can make robot class
        dxl_io.close()

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

        # do control loop for setting new positions

    def control_main_ROS(self):
        loop_rate = rospy.Rate(10)   

        # plan from human and arm position

        # move motors

            loop_rate.sleep() # 20 Hz
