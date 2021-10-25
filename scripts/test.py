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

time.sleep(5)