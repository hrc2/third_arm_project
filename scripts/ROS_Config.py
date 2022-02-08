import numpy as np

class ROS_config:
    
    def __init__(self):
        self.opti_track_origin = "optitrack_origin"
        self.third_arm_base = "third_arm_base"
        self.third_arm_other_hand = "third_arm_other_hand"
        self.third_arm_gripper = "third_arm_gripper"

        self.arm_on = True
        self.IK_tolerance = .1


        self.base_offset_for_IK = np.array(
        [[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0.1],
        [0, 0, 0, 0]])