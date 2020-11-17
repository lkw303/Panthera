#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import getch
import serial

width = 0.56
length = 1
rad = 0.5
switch = True

port = '/dev/ttyUSB1'

def adjust_wheels(radius): # radius in m, direction c(-1) or ccw(1)
	if radius == 0:
		left_back = round(math.degrees(math.atan((length*0.5) / (width/2))), 2)
		left_front = -left_back

		right_back = left_front
		right_front = left_back

		return [left_back, right_back, left_front, right_front]

	else:
		r_inner = radius - width/2
		r_outer = radius + width/2
		
		inner_front = round(math.degrees(math.atan((length*0.5) / r_inner)), 2)
		inner_back = -inner_front

		outer_front = round(math.degrees(math.atan((length*0.5) / r_outer)), 2)
		outer_back = -outer_front

		return [inner_front, inner_back, outer_front, outer_back]

def run(x):
	global switch
	twist = Twist()
	#signal = getch.getch()

	if x == 'w':
		ls = adjust_wheels(float('inf'))
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		pub.publish(twist)

	if x == 'e':
		ls = adjust_wheels(rad)
		twist.linear.x = -ls[3]
		twist.linear.y = -ls[1]
		twist.linear.z = -ls[2]
		twist.angular.x = -ls[0]
		pub.publish(twist)

	if x == 'q':
		ls = adjust_wheels(rad)
		twist.linear.x = ls[1]
		twist.linear.y = ls[3]
		twist.linear.z = ls[0]
		twist.angular.x = ls[2]
		pub.publish(twist)

	if x == 'd':
		twist.linear.x = -90
		twist.linear.y = -90
		twist.linear.z = -90
		twist.angular.x = -90
		pub.publish(twist)

	if x == 'a':
		twist.linear.x = 90
		twist.linear.y = 90
		twist.linear.z = 90
		twist.angular.x = 90
		pub.publish(twist)

	if x == 's':
		ls = adjust_wheels(0)
		twist.linear.x = ls[0]
		twist.linear.y = ls[1]
		twist.linear.z = ls[2]
		twist.angular.x = ls[3]
		pub.publish(twist)

	if x == 'p':
		switch = False
        print(twist)

if __name__ == "__main__":
	try:
		rospy.init_node('controller')
		ser = serial.Serial(port)
		pub = rospy.Publisher('target_angle', Twist, queue_size=1)
		while switch:
			cmd = ser.readline().strip('\r\n')
			print(cmd)
			run(cmd)
	except rospy.ROSInterruptException:
		pass

