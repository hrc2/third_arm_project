#!/usr/bin/env python  
import rospy
import roslib
import math
import tf
import csv
import arbotix_msgs.srv
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def make_marker(trans,rgba):
	marker = Marker()
	marker.header.frame_id = "human/base"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.05
	marker.scale.y = 0.05
	marker.scale.z = 0.05
	marker.color.a = rgba[3]
	marker.color.r = rgba[0]
	marker.color.g = rgba[1]
	marker.color.b = rgba[2]
	marker.pose.orientation.w = 1.0
	marker.pose.position.x = trans[0]
	marker.pose.position.y = trans[1] 
	marker.pose.position.z = trans[2] 
	marker.lifetime = rospy.Duration(0)
	return marker

if __name__ == '__main__':	

	rospy.init_node('thirdarm_ws')
	listener = tf.TransformListener()
	topic = 'marker_array'
	publisher = rospy.Publisher(topic, Marker, queue_size=100)
	count = 0
	MARKERS_MAX = 10000
	markerArray = MarkerArray()
	#rate = rospy.Rate(5)
	#while not rospy.is_shutdown():
	#rospy.Subscriber('/joint_states', JointState, callback)

	rate = rospy.Rate(10.0)
	
	while not rospy.is_shutdown():
		try:
		    (trans,rot) = listener.lookupTransform('human/base', 'gripper_motor', rospy.Time(0))
		    (htrans,hrot) = listener.lookupTransform('human/base', 'human/right_wrist', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		rospy.loginfo('Position of gripper motor {0}'.format(trans))
		
		# with open('v2_new.csv','a') as f:
		# 	writer = csv.writer(f)
		# 	writer.writerow(trans)

		with open('human.csv','a') as fh:
			writer = csv.writer(fh)
			writer.writerow(htrans)

		rgb = [0.0,1.0,0.0,0.8]
		marker = make_marker(trans,rgb)

		hrgb = [1.0,0.615,0.0,0.65]
		hmarker = make_marker(htrans,hrgb)

		# We add the new marker to the MarkerArray, removing the oldest
		# marker from it when necessary
		if(count > MARKERS_MAX):
		   markerArray.markers.pop(0)

		markerArray.markers.append(marker)
		markerArray.markers.append(hmarker)
		# Renumber the marker IDs
		id = 0
		for m in markerArray.markers:
		   m.id = id
		   id += 1

		# Publish the MarkerArray
		publisher.publish(marker)
		publisher.publish(hmarker)
		print 'PosX {0}'.format(trans[0])

		count += 1
		rate.sleep()

	#rospy.spin()
	#rate.sleep()
	