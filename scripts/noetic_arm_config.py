class base_swivel:
    id = 1
    max = 0
    min = 4000
    range = abs(max-min)
    back = 0
    forward = 2000
    left = 1000
    right = 3000

class vertical_tilt:
    id = 2
    max = 3500
    min = 0
    range = abs(max-min)     

class arm_extension:
    id = 3
    max = 1850
    min = 0
    range = abs(max-min)     

class wrist:
    id = 4
    max = 1000
    min = 0
    range = abs(max-min)
    # 500 - rotate right
    # level, upright- 0
    # max - 1000, level upright

class wrist_tilt:
    id = 5
    max = 1000
    min = 0
    range = abs(max-min)
    level = 500

class gripper:
    id = 6
    max = 2700
    min = 2000
    range = abs(max-min)
    open = 2000
    closed = 2700

joint_names = [base_swivel, vertical_tilt, arm_extension, wrist, wrist_tilt, gripper]