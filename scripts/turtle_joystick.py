#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    # vertical left stick axis = linear rate
    twist.linear.x = 4*data.axes[1]
    # horizontal left stick axis = turn rate
    twist.angular.z = 4*data.axes[0]
    pub.publish(twist)

# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    rospy.init_node('Joy2Turtle')
    global pub
    #r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 10)
    # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, callback)
    # starts the node
     #   r.sleep()   
        rospy.spin()

if __name__ == '__main__':
    start()

