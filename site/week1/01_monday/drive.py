#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

AUTONOMOUS_MODE = False

class Drive(object):
    def __init__(self):
		rospy.init_node("drive")
		self.data = None
		self.cmd = AckermannDriveStamped()
		self.laser_sub = rospy.Subscriber("scan", LaserScan, self.scan, queue_size=1)
		self.drive_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=1)
		self.joy_sub = rospy.Subscriber("/vesc/joy", Joy, self.joystick_callback, queue_size=1)

    def joystick_callback(self, joy):
        '''Resets AUTONOMOUS_MODE to T/F for emergency stopping'''
        global AUTONOMOUS_MODE
        if int(joy.buttons[4]) == 1:
            AUTONOMOUS_MODE = False
        if int(joy.buttons[1]) == 1:
            AUTONOMOUS_MODE = True

    def scan(self, data):
		self.data = data
		self.drive_callback()

    def drive_callback(self):
                #WRITE YOUR CODE HERE

if __name__ == "__main__":
	try:
		node = Drive()
		while not rospy.is_shutdown():
			if AUTONOMOUS_MODE:
				node.drive_pub.publish(node.cmd)
	except rospy.ROSInterruptException:
		exit()
