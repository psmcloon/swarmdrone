#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from scipy.spatial.transform import Rotation as R
import numpy as np

def getpose(data):

	translation = [[]]
	rotation = [[]]

	for detection in data.detections:

		posx = detection.pose.pose.pose.position.x
		posy = detection.pose.pose.pose.position.y
		posz = detection.pose.pose.pose.position.z

		quater_rotx = detection.pose.pose.pose.orientation.x
		quater_roty = detection.pose.pose.pose.orientation.y
		quater_rotz = detection.pose.pose.pose.orientation.z
		quater_rotw = detection.pose.pose.pose.orientation.w

		quatermatrix = [quater_rotx, quater_roty, quater_rotz, quater_rotw]

		rotation = R.from_quat(quatermatrix)
		rotation = rotation.as_matrix()

		translation = [[posx, posy, posz]]

	return translation, rotation

def callback(data):

	translation, rotation = getpose(data)
	print("Translation Matrix: ", translation)
	print("Rotation Matrix: ", rotation)

	# AprilTag script goes here

def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber('tag_detections', AprilTagDetectionArray, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()

