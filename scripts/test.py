from pypot.dynamixel import DxlIO
import time

port = "/dev/ttyACM0"
dxl_io = DxlIO(port, baudrate=1000000)
# print(dxl_io.scan(range(10)))


dxl_io.set_goal_position({6: 100})
dxl_io.set_goal_position({5: -50})
dxl_io.set_goal_position({4: 0})

time.sleep(2)
dxl_io.set_goal_position({5: 50})
dxl_io.set_goal_position({6: 0})
dxl_io.set_goal_position({4: 50})

time.sleep(2)

old_angle = dxl_io.get_angle_limit(5)
print(old_angle)
dxl_io.set_wheel_mode((5,))  # wheel mode
dxl_io.set_moving_speed({5: 2})
dxl_io.set_joint_mode((5,))
dxl_io.set_goal_position({5: 50})
time.sleep(1)
dxl_io.set_goal_position({5: 0})
print(dxl_io.get_angle_limit(5))

