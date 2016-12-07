#!/usr/bin/env python  
import rospy
import roslib
import math
import arbotix_msgs.srv
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import JointState

class ThirdArm():
    def __init__(self):
        rospy.init_node('third_arm_kinect')
        self.base_pose = 0.0
        self.elbow_pose = 0.0
        self.wrist_pose = 0.0
        self.arm_pose = 0.0
        self.hand_pose = 0.0
        self.is_arbotix = 0
        self.direction = Vector3(0,0,0) #Direction is gotten from the Kinect
        self.error = None
        self.set_point = Vector3(0,0,0)
        self.init_pubs()
        self.base_speed_max = 0.157

    def arm_callback(self,msg): 
        self.elbow_pose = msg.position[0]
        self.base_pose = msg.position[1]
        self.wrist_pose = msg.position[2]
        self.arm_pose = msg.position[3]
        self.hand_pose = msg.position[4]
        #Check if arbotix is connected
        if msg:
            self.is_arbotix = 1 

    def kinect_callback(self,msg):       
        self.direction = Vector3(msg.x,msg.y,msg.z)

        
    def init_pubs(self):
        # Publishers to the joint states topics:
        # pub to arm to move it
        # need to launch arbotix_driver, pub to servo/command, type Float64
        self.pub_base = rospy.Publisher('/base/command', Float64, queue_size=1)
        self.pub_elbow = rospy.Publisher('/elbow/command', Float64, queue_size=1)
        self.pub_arm = rospy.Publisher('/arm/command', Float64, queue_size=1)
        self.pub_wrist = rospy.Publisher('/wrist/command', Float64, queue_size=1)
        self.pub_hand = rospy.Publisher('/hand/command', Float64, queue_size=1)


    def closed_loop_calculate(self):
        self.error_old = self.error

        x = self.direction.x
        y = self.direction.y
        z = self.direction.z

        self.error = (x**2 + y**2)**0.5
        scale = math.pi

        Kp = 1

        if self.error_old > self.error:
            Kp = -Kp
            self.base_command = Kp*self.error
        else:
            self.base_command = Kp*self.error

        scale = 1.5
        self.base_command = self.base_command*self.base_speed_max/scale

        # if math.fabs(self.base_command) < 0.05:
        #     self.base_command = 0

        # self.Kp = 1
        # self.Kd = 0
        # self.Ki = 0
        # self.Integrator_max= 10
        # self.Integrator_min= -10
        
        # self.P_value = self.Kp * self.error
        # self.D_value = self.Kd * ( self.error - self.Derivator)
        # self.Derivator = self.error

        # self.Integrator = self.Integrator + self.error

        # if self.Integrator > self.Integrator_max:
        #     self.Integrator = self.Integrator_max
        # elif self.Integrator < self.Integrator_min:
        #     self.Integrator = self.Integrator_min

        # self.I_value = self.Integrator * self.Ki

        # self.PID = self.P_value + self.I_value + self.D_value



    def closed_loop_run(self):
        self.rate = rospy.Rate(10)


        while not rospy.is_shutdown():
                    # set up set_speed service

            # Subscribe to direction from the Kinect
            rospy.Subscriber('/direction', Vector3, self.kinect_callback)
            # Subscribe to current servo positions
            rospy.Subscriber('/joint_states', JointState, self.arm_callback)
            #As of now, just using one motor: the base swivel

            rospy.wait_for_message('/direction', Vector3)
            if self.is_arbotix:
                self.base_speed = rospy.ServiceProxy('/base/set_speed', arbotix_msgs.srv.SetSpeed)
                self.base_speed(self.base_speed_max)
            # print (elbow_pose, base_pose, wrist_pose, arm_pose, hand_pose)                 
                self.closed_loop_calculate()
                rospy.loginfo("The base command is : {0}".format(self.base_command))
                self.pub_base.publish(0.0)
                self.pub_elbow.publish(0.0)
                self.pub_arm.publish(0.0)
                self.pub_wrist.publish(0.0)
                self.pub_hand.publish(0.0)


        self.rate.sleep()



if __name__ == '__main__':
    try:
        #rospy.init_node('third_arm_kinect')
        
        third_arm = ThirdArm()
        third_arm.closed_loop_run()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Position Calculator node terminated.")






