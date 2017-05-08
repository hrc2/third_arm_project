#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Bool

# Define the publisher here
pub = rospy.Publisher('chatterer', String, queue_size=50)

# joint position controllers
wrist_pub = rospy.Publisher('/wrist_controller/command', Float64, queue_size=10)
arm_extension_pub = rospy.Publisher('/arm_extension_controller/command', Float64, queue_size=10)
dyn_pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)
vt_pub = rospy.Publisher('/vertical_tilt_controller/command',Float64, queue_size=10)
bs_pub = rospy.Publisher('/base_swivel_controller/command',Float64, queue_size=10)
grip_pub = rospy.Publisher('/gripper_controller/command',Float64, queue_size=10)

# joint torque controllers
bs_t_pub = rospy.Publisher('/base_swivel_controller_t/command',Float64, queue_size=1)
vt_t_pub = rospy.Publisher('/vertical_tilt_controller_t/command',Float64, queue_size=1)
arm_extension_t_pub = rospy.Publisher('/arm_extension_controller_t/command',Float64, queue_size=50)
wrist_t_pub = rospy.Publisher('/wrist_controller_t/command',Float64, queue_size=1)
grip_t_pub = rospy.Publisher('/gripper_controller_t/command',Float64, queue_size=1)


# Define global variables to be used
x_vel = 0
grip_state = False

# Create a callback function for the subscriber
def callback(data):

    # Get Orientation information from the controller
    orientation = QuatToEuler(data.orientation) # [[roll, pitch, yaw] in rad/s
    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]

    # Get linear acceleration of the controller
    lin_accel = data.linear_acceleration
    x_accel = lin_accel.x
    y_accel = lin_accel.y
    z_accel = lin_accel.z

    # Print data for debugging
    #rospy.loginfo("IMU roll %s, pitch %s, yaw %s ", roll,pitch,yaw)
    #rospy.loginfo("IMU accel x %s, accel y %s, accel z %s ", x_accel,y_accel,z_accel)
    

    #Control arm extension
    extension_velocity

    if extension_velocity is None:
        global extension_velocity

    if count is None:
        global count
        count = 0

    count = count+1
    # Logic for arm extension TODO explain this further Lauren TODO
    if extension_velocity==0 and x_accel>.7 and count>10:
        extension_velocity=  -.5
        count = 0
    elif extension_velocity==0 and x_accel<-.7 and count>10:
        extension_velocity= .5
        count = 0
    elif (extension_velocity==.5 or extension_velocity==-.5) and count >10:
        if x_accel>.5 or x_accel<-.5:
            extension_velocity=0
            count = 0

    arm_extension_t_pub.publish(extension_velocity)
    #rospy.loginfo("%s", extension_velocity)
    #rospy.loginfo("%s", count)

    # Control the wrist rotation with the roll of the controller
    wrist_cmd = roll
    wrist_pub.publish(wrist_cmd)

    #pub_str =  "%s" % yaw
    #pub.publish(pub_str)

    # Control the arm extension with x acceleration
    #global pub_exten # declare that we want to modify the global variable
    #global x_vel

    #vel = calcVelocity(x_accel) # calculate the velocity

    #if x_accel > 1:
    #    pub_exten =pub_exten-.1
    #if x_accel < -1:
    #    pub_exten =pub_exten+.1
    #if vel < -0.5:
    #    pub_exten = 0
    #arm_extension_pub.publish(pub_exten)

    # control the base swivel angle with the yaw of the controller
    bs_cmd = -yaw
    bs_pub.publish(bs_cmd)

    # Control the vetical tilt with the pitch of the controller
    vt_cmd = pitch
    # if the third_arm is under neath the wearer's arm, do not allow it to tilt upwards
    #if (bs_cmd < 0.5 and bs_cmd > -0.5):
    #    if vt_cmd > 0:
     #       vt_cmd = 0
    vt_pub.publish(vt_cmd)

def grip_callback(data):
    global grip_state # when true, gripper is closed
    button_state = data.data
    rospy.loginfo("%s", button_state)

    if(button_state and grip_state): # if button pressed and gripper closed, open it
        grip_cmd = 0
        grip_pub.publish(grip_cmd)
        grip_state = False
    elif(button_state and not grip_state): # if button pressed and gripper open, close it
        grip_cmd = 1.4
        grip_pub.publish(grip_cmd)
        grip_state = True

    rospy.loginfo("%s", grip_state)


    
# Main while loop
def control():

    global extension_velocity
    extension_velocity = 0

    global count
    count = 0
    # Intialize the node and name it
    rospy.init_node('translator', anonymous=True)

    # initialize the arm to 0 positions


    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.Subscriber("imu", Imu, callback)
    rospy.Subscriber("grip", Bool, grip_callback)
    rospy.loginfo("subscribers started")

    # now run publisher for the rest of the time
    # subscriber loop is handled in the callback thread

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

    return [roll,pitch,yaw]

def calcVelocity(accel):
    
    global x_vel

    x_vel = x_vel + accel*0.1

    return x_vel


if __name__ == '__main__':
    # Go to the main loop
    try:
        control()
    except rospy.ROSInterruptException:
        pass