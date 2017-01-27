#!/usr/bin/env python  
import rospy
import roslib
import math
import tf
import arbotix_msgs.srv
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# i = 0.0
# def callback(data):
# 	pub = rospy.Publisher('/third_arm_joints', JointState, queue_size=1)
# 	global i
# 	i += 0.01
# 	state = data
# 	val = 1.57*math.sin(i)
# 	state.position = [val,val,math.fabs(0.1*val),val,val,val]
	
# 	# if i < 1.57 and incr == 1:
# 	# 	i += 0.01
# 	# elif incr == -1:
# 	# 	i -= 0.01

# 	# if i>1.57:
# 	# 	incr = -1
# 	# elif i<0:
# 	# 	incr = 1

# 	pub.publish(state)

if __name__ == '__main__':	

	rospy.init_node('thirdarm_ws')
	listener = tf.TransformListener()
	count = 0
	MARKERS_MAX = 100
	markerArray = MarkerArray()
	#rate = rospy.Rate(5)
	#while not rospy.is_shutdown():
	#rospy.Subscriber('/joint_states', JointState, callback)

	rate = rospy.Rate(10.0)
	
	while not rospy.is_shutdown():
		try:
		    (trans,rot) = listener.lookupTransform('wrist_motor', 'human/base', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

    	rospy.loginfo('Position of wrist motor {0}'.format(trans))
    	rate.sleep()

		marker = Marker()
		marker.header.frame_id = "/neck"
		marker.type = marker.POINTS
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = math.cos(count / 50.0)
		marker.pose.position.y = math.cos(count / 40.0) 
		marker.pose.position.z = math.cos(count / 30.0) 

		# We add the new marker to the MarkerArray, removing the oldest
		# marker from it when necessary
		if(count > MARKERS_MAX):
		   markerArray.markers.pop(0)

		markerArray.markers.append(marker)

		# Renumber the marker IDs
		id = 0
		for m in markerArray.markers:
		   m.id = id
		   id += 1

		# Publish the MarkerArray
		publisher.publish(markerArray)

		count += 1

	#rospy.spin()
	#rate.sleep()
	