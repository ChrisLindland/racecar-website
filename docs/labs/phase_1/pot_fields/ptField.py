#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

AUTONOMOUS_MODE = False

class PotentialField:
	DRIVE_TOPIC = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
	SCAN_TOPIC = "/scan"

	def __init__(self):
		rospy.init_node("potentialField")
		self.data = None
		self.cmd = AckermannDriveStamped()
		
		#cartesian points -- to be filled (tuples)
		self.cartPoints = [None for x in range(1081)]
		
		#[speed, angle]
		self.finalVector = [0.5, 0]
                
        #caps to make sure the car doesn't go wild
		self.maxSpeed = 1
		self.maxTurn = 1
    
	'''def joystick_callback(self, joy):
		Resets AUTONOMOUS_MODE to T/F for emergency stopping
		global AUTONOMOUS_MODE
		if int(joy.buttons[4]) == 1:
			AUTONOMOUS_MODE = False
		if int(joy.buttons[1]) == 1:
			AUTONOMOUS_MODE = True'''

	def scan_callback(self, data):
		'''Checks LIDAR data'''
		self.data = data.ranges
		self.drive_callback()

	def drive_callback(self):
		'''Publishes drive commands'''

	def convertPoints(self, points):
		'''Convert all current LIDAR data to cartesian coordinates'''

	def calcFinalVector(self, points):
		'''Calculate the final driving speed and angle'''

if __name__ == "__main__":
	try:
		node = PotentialField()
		while not rospy.is_shutdown():
			if AUTONOMOUS_MODE:
				node.drive_pub.publish(node.cmd)
	except rospy.ROSInterruptException:
		exit()

    


