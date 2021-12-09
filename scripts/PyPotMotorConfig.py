from pypot.dynamixel import DxlIO

motors_types = {
    1: 'MX_64',  # https://emanual.robotis.com/docs/en/dxl/mx/mx-64/
    2: 'MX_64',  # ^
    3: 'MX_28',  # https://emanual.robotis.com/docs/en/dxl/mx/mx-28/
    4: 'AX_12A',  # https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
    5: 'AX_12A',  # ^
    6: 'MX_28'  # ^^^
}

third_arm_robot_config = {
    'controllers': {
        'my_dxl_controller': {
            'sync_read': False,
            'attached_motors': ['wrist', 'arm'],
            'port': 'auto'
        }
    },
    'motorgroups': {
        'wrist': ['gripper', 'wrist_tilt', 'wrist_axial'],
        'arm': ['base_swivel', 'vertical_tilt', 'arm_extension']
    },
    'motors': {
        'gripper': {
            'orientation': 'direct',
            'id': 6,
            'type': 'MX_28',
            'angle_limit': (-4.18, 57.36),
            'offset': 0.0,
            'moving_speed': 20,
        },
        'wrist_tilt': {
            'orientation': 'direct',
            'id': 5,
            'type': 'AX_12A',
            'angle_limit': (-103.08, 90.47),
            'offset': 0.0,
            'moving_speed': 80,
        },
        'wrist_axial': {
            'orientation': 'direct',
            'id': 4,
            'type': 'AX_12A',
            'angle_limit': (-97.21, 84.6),
            'offset': 0.0,
            'moving_speed': 80,
        },
        'arm_extension': {
            'orientation': 'direct',
            'id': 3,
            'type': 'MX_28',
            'angle_limit': (-180.0, 0.04),
            'offset': 0.0,
            'moving_speed': 20,
        },
        'vertical_tilt': {
            'orientation': 'direct',
            'id': 2,
            'type': 'MX_64',
            'angle_limit': (-180.0, 121.63),
            'offset': 0.0,
            'moving_speed': 20,
        },
        'base_swivel': {
            'orientation': 'direct',
            'id': 1,
            'type': 'MX_64',
            'angle_limit': (-180.0, 180.0),
            'offset': 0.0,
            'moving_speed': 20,
        }
    }
}

# assemble config to get motor min and maxes to be used when controlling
control_config = {}
for motor in third_arm_robot_config['motors']:
    angle_limit = third_arm_robot_config['motors'][motor]['angle_limit']
    control_config[motor] = {}  # create dictionary for controlling motor max, min
    control_config[motor]['max'] = angle_limit[1]
    control_config[motor]['min'] = angle_limit[0]


def get_motor_config(motor_number, motor_name):
    """ use to get the motor config from the robot
    
    Args:
        motor_number: motor number that want to get data from
        motor_name: name of motor getting data from    
    """

    port = "/dev/ttyACM0"
    dxl_io = DxlIO(port, baudrate=1000000)

    angle_limits = dxl_io.get_angle_limit((motor_number,))[0]
    motor_type = motors_types[motor_number]

    print("'" + motor_name + "':{")
    print("'orientation': 'direct',")
    print("'id':" + str(motor_number) + ",")
    print("'type':'" + str(motor_type) + "',")
    print("'angle_limit':" + str(angle_limits) + ",")
    print("'offset': 0.0,")
    print("}")
