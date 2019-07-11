#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class PotentialField:
	SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
	DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")

	def __init__(self):
		rospy.init_node("potentialField")
		self.data = None
		self.cmd = AckermannDriveStamped()

		#write your publishers and subscribers here; they should be the same as the wall follower's
		
		#cartesian points -- to be filled (tuples)
		self.cartPoints = [None for x in range(1081)]
		
		#[speed, angle]
		self.finalVector = [0.5, 0]

	def scan_callback(self, data):
		'''Checks LIDAR data'''
		self.data = data.ranges
		self.drive_callback()

	def drive_callback(self):
		'''Publishes drive commands'''
		#make sure to publish cmd here

	def convertPoints(self, points):
		'''Convert all current LIDAR data to cartesian coordinates'''

	def calcFinalVector(self, points):
		'''Calculate the final driving speed and angle'''

if __name__ == "__main__":
	rospy.init_node('potential_field')
	potential_field = PotentialField()
	rospy.spin()

    


