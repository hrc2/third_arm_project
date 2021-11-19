
import time
from PyPotMotorConfig import motors_types


from contextlib import closing

import pypot.robot
from pypot.dynamixel import DxlIO, motor

from PyPotMotorConfig import third_arm_robot_config, control_config


port = "/dev/ttyACM0"
dxl_io = DxlIO(port, baudrate=1000000)
# print(dxl_io.scan(range(10)))
motor_positions = dxl_io.get_present_position([6])
print(motor_positions)    

dxl_io.set_goal_position({6: 100})
# dxl_io.set_goal_position({5: -50})
# dxl_io.set_goal_position({4: 0})

time.sleep(2)
# dxl_io.set_goal_position({5: 50})
dxl_io.set_goal_position({6: 0})
# dxl_io.set_goal_position({4: 50})


