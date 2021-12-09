

class motion_planning_controller_config:
    """config class for motion_planning_controller"""

    def __init__(self):
        self.port = "/dev/ttyACM0"
        self.baudrate = 1000000
        self.pid_velocity_threshold = 0.1
        self.max_velocity = 1

        self.kp_base_swivel = 1
        self.ki_base_swivel = 0
        self.kd_base_swivel = 0

        self.kp_vertical_tilt = 1
        self.ki_vertical_tilt = 0
        self.kd_vertical_tilt = 0

        self.kp_arm_extension = 1
        self.ki_arm_extension = 0
        self.kd_arm_extension = 0

        self.kp_wrist_axial = 1
        self.ki_wrist_axial = 0
        self.kd_wrist_axial = 0

        self.kp_wrist_tilt = 1
        self.ki_wrist_tilt = 0
        self.kd_wrist_tilt = 0

        self.kp_gripper = 1
        self.ki_gripper = 0
        self.kd_gripper = 0


