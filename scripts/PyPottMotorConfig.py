import MAIN_CONFIG_CLASSES
from pypot.dynamixel import DxlIO

third_arm_robot_config = {
    'controllers': {
        'my_dxl_controller': {
            'sync_read': False,
            'attached_motors': ['base', 'tip'],
            'port': '/dev/ttyACM0'
        }
    },
    'motorgroups': {
        'wrist': ['gripper', 'wrist_tilt', 'wrist_axiel'],
        'arm': ['base_swivel', 'vertical_tilt', 'arm_extension']
    },
    'motors': {
    }
}

def get_motor_config(motor_number, motor_name):
    port = "/dev/ttyACM0"
    dxl_io = DxlIO(port, baudrate=1000000)

    angle_limits = dxl_io.get_angle_limit((motor_number,))[0]
    motor_type = MAIN_CONFIG_CLASSES.motors[motor_number]

    print("'"+motor_name+"':{")
    print("'orientation': direct,")
    print("'type':"+motor_type+",")
    print("'angle_limit':" + angle_limits + ",")
    print("'offset': 0.0,")
    print("}")
