#! /usr/bin/env python3
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import rospy
import tf
from scipy.spatial.transform import Rotation as R
import numpy as np

#local imports
from inverse_kinematics import InverseKinematicsSolver

class WRTA_Gazebo_Controller:
    '''
    Subscribe to Cartesian Pose commands for the gripper.

    '''
    def __init__(self):
        rospy.init_node('wrta_gazebo_controller')
        self.pose_subscriber = rospy.Subscriber('/wrta/pose_command', Pose, self.pose_callback)
        self.tf_listener = tf.TransformListener()
        self.IKSolver = InverseKinematicsSolver()
        self.joint_position_publishers = [
            rospy.Publisher('/wrta/horizontal_panning_controller/command', Float64, queue_size=1),
            rospy.Publisher('/wrta/vertical_pitching_controller/command', Float64, queue_size=1),
            rospy.Publisher('/wrta/length_extension_controller/command', Float64, queue_size=1),
            rospy.Publisher('/wrta/wrist_pitching_controller/command', Float64, queue_size=1),
            rospy.Publisher('/wrta/wrist_rotation_controller/command', Float64, queue_size=1)
        ]
        

    def pose_callback(self,msg):
        '''
        When we get a pose (position and orientation) command for the gripper,
        -call IK solver to get joint angles
        -call the motor controllers to send new joint angles
        '''
        pose_matrix = self.get_transform_from_translation_and_rotation([msg.position.x,msg.position.y,msg.position.z],[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        valid, joint_targets = self.IKSolver.solve_kinematics(pose_matrix)
        if valid:
            rospy.loginfo(f"Publishing Joint Targets: {joint_targets} ")
            for i,target in enumerate(joint_targets):
                target_msg = Float64()
                target_msg.data = target
                self.joint_position_publishers[i].publish(target_msg)
        else:
            rospy.logerr(f"Invalid: {joint_targets} ")

    def get_transform_from_translation_and_rotation(self, translation, rotation):
            """ get the transformation matrix from the tranlsation and rotation arrays """

            # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html
            # http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
            # https://stackoverflow.com/questions/40833073/insert-matrix-into-the-center-of-another-matrix-in-python

            rotation_matrix = R.from_quat(rotation)
            # rotation_matrix = rotation_matrix.as_dcm()
            rotation_matrix = rotation_matrix.as_matrix()

            # assemble transform
            transform = np.zeros((4, 4))

            # add rotation
            transform[0:3, 0:3] = rotation_matrix

            # add translation
            translation = np.array(translation)
            transform[0:3, 3:4] = translation.reshape((3, 1))

            transform[3][3] = 1

            return transform

if __name__=='__main__':
    gpc = WRTA_Gazebo_Controller()
    rospy.spin()