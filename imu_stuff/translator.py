#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

# Define the publisher here
pub = rospy.Publisher('chatterer', String, queue_size=10)
dyn_pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)

# Create a callback function for the subscriber
def callback(data):
    # Print the calculated angle to the terminal (for debugging)
    yaw = QuatToEuler(data.orientation) # yaw data in rad

    rospy.loginfo("IMU yaw at %s rad", yaw)

    pub_str =  "%s" % yaw
    pub.publish(pub_str)

    dyn_com = yaw + 3.14
    dyn_pub.publish(dyn_com)

    
# Main while loop
def control():


    # Intialize the node and name it
    rospy.init_node('translator', anonymous=True)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.Subscriber("imu", Imu, callback)
    rospy.loginfo("subscriber started")

    # now run publisher for the rest of the time
    # subscriber loop is handled in the callback thread
    #rate = rospy.Rate(10) # 10hz refresh rate for the loop
    #while not rospy.is_shutdown():
    #    hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
    #    pub.publish(hello_str)
     #   rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def QuatToEuler(Quat):
    q1 = Quat.x
    q2 = Quat.y
    q3 = Quat.z
    q4 = Quat.w
    q = Quat

    ysqr = q.y * q.y

    # roll (x-axis rotation)
    t0 = 2.0 * (q.w * q.x + q.y * q.z)
    t1 = 1.0 - 2.0 * (q.x * q.x + ysqr)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = 2.0 * (q.w * q.y - q.z * q.x)
    if t2 > 1.0: 
        t2 = 1.0
    if t2 < -1.0:
        t2 = -1.0
    pitch = math.asin(t2);

    # yaw (z-axis rotation)
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (ysqr + q.z * q.z)  
    yaw = math.atan2(t3, t4)

    return yaw

if __name__ == '__main__':
    # Go to the main loop
    try:
        control()
    except rospy.ROSInterruptException:
        pass