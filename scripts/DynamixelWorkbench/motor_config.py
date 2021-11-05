class base_swivel:
    motor_id = 1
    max = 0
    min = 4000
    range = abs(max-min)
    back = 0
    forward = 2000
    left = 1000
    right = 3000

class vertical_tilt:
    motor_id = 2
    max = 3500
    min = 0
    range = abs(max-min)     

#Extension should be in multi-turn mode
class arm_extension:
    motor_id = 3
    max = 1850
    min = 0
    range = abs(max-min)     

class wrist_axiel:
    motor_id = 4
    max = 1000
    min = 0
    range = abs(max-min)
    # 500 - rotate right
    # level, upright- 0
    # max - 1000, level upright

class wrist_tilt:
    motor_id = 5
    max = 1000
    min = 0
    range = abs(max-min)
    level = 500

class gripper:
    motor_id = 6
    max = 2700
    min = 2000
    range = abs(max-min)
    open = 2000
    closed = 2700

joint_names = [base_swivel, vertical_tilt, arm_extension, wrist_axiel, wrist_tilt, gripper]