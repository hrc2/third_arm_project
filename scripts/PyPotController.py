from contextlib import closing

import pypot.robot
from pypot.dynamixel import DxlIO

# The closing decorator make sure that the close function will be called
# on the object passed as argument when the with block is exited.

with closing(pypot.robot.from_json('myconfig.json')) as my_robot:
    # do stuff without having to make sure not to forget to close my_robot!
    pass

    port = "/dev/ttyACM0"
    dxl_io = DxlIO(port, baudrate=1000000)
    old_angle =
    dxl_io.set_wheel_mode() # wheel mode
    dxl_io.set_moving_speed() # now set the moving speed of the mtor