#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Joy, Imu
from ackermann_msgs.msg import AckermannDriveStamped
import sys, math, random
import decimal
from random import *

AUTONOMOUS_MODE = False

class WallFollow(object):
	def __init__(self):
		rospy.init_node("wallFollow")
		#LIDAR data
		self.data = None
		self.cmd = AckermannDriveStamped()

        #add publishers and subscribers
                
	def joystick_callback(self, joy):
		'''Resets AUTONOMOUS_MODE to T/F for emergency stopping'''
		global AUTONOMOUS_MODE
		if int(joy.buttons[4]) == 1:
		    AUTONOMOUS_MODE = False
		if int(joy.buttons[1]) == 1:
		    AUTONOMOUS_MODE = True

	def scan_callback(self, data):
		'''Checks LIDAR data'''
		self.data = data
		self.drive_callback()

	def drive_callback(self):
		'''Assign the driving angle based on the wall'''
	 
	def convertPoint(self, mag, direction):
		'''Converts a polar (LIDAR) point to cartesian coordinates'''

	def estimateWall(self, direction):
		'''Predicts a wall. Direction is either right or left. Returns the points used to find the wall'''
		
	def chooseWall(self):
		'''Chooses which wall, right or left, to follow'''

if __name__ == "__main__":
	try:
		node = WallFollow()
		rospy.Rate(100)
		while not rospy.is_shutdown():
			if AUTONOMOUS_MODE:
				node.drive_pub.publish(node.cmd)
	except rospy.ROSInterruptException:
		exit()
