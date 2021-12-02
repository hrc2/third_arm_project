

class motion_planning_controller_config:
    """config class for motion_planning_controller"""

    def __init__(self):
        self.port = "/dev/ttyACM0"
        self.baudrate = 1000000
        self.kp_base_swivel = 1
        self.pid_velocity_threshold = 0.1
        self.max_velocity = 1