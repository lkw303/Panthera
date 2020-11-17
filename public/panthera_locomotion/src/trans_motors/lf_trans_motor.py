#!/usr/bin/env python

import rospy
import math
import orienbus
from geometry_msgs.msg import Twist
import serial.tools.list_ports

############## Change for different motor ###############
address = 2
p = list(serial.tools.list_ports.grep(rospy.get_param('/lf_sn')))
port = '/dev/' + p[0].name
#########################################################

orienbus = orienbus.OrienBus(port)
motor = orienbus.initialize(address)

width = 1.02
length = 1.31
wheel_radius = 0.1

####### Check if gear ratio correct #######
gear_ratio = 10

def reconfig(data): ###
	global wheel_speed
	wheel_speed = data.linear.z

def callback(data):
	global linear_x
	global angular_z
	linear_x = data.angular.y
	angular_z = data.angular.z

def motor_lin_vel(vx, wz):
	sign = wz / abs(wz)
	r = abs(vx / wz)
	speed = math.sqrt((r - sign*width/2)**2 + (length/2)**2) * abs(wz)
	return speed # check motor direction 

def rads_to_rpm(x):
	rpm = (x / (2*math.pi)) * 60 * gear_ratio
	return int(-rpm)

def adjust_speed(vx, wz):
	speed = 0
	if vx == 0:
		if wz == 0:
			motor.writeSpeed(0)
			print("lf rpm: 0")

		else:
			speed = -wz * math.sqrt((length/2)**2 + (width/2)**2) / wheel_radius
			rpm = rads_to_rpm(speed)
			print("lf rpm: " + str(rpm))
			motor.writeSpeed(rpm)			

	else:
		if wz == 0:
			speed = vx / wheel_radius
			rpm = rads_to_rpm(speed)
			print("lf rpm: " + str(rpm))
			motor.writeSpeed(rpm)

		else:
			lin_vel  = motor_lin_vel(vx, wz)
			speed = lin_vel / wheel_radius

			rpm = rads_to_rpm(speed)
			print("lf rpm: " + str(rpm))
			motor.writeSpeed(rpm)

if __name__ == "__main__":
	try:
		linear_x = 0
		angular_z = 0
		wheel_speed = 0

		########## Change node name #############
		rospy.init_node('lf_trans_motor')

		sub = rospy.Subscriber('target_angle', Twist, callback)

		sub2 = rospy.Subscriber('reconfig', Twist, reconfig)
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			if wheel_speed == 0:
				adjust_speed(linear_x, angular_z)

			else:
				motor.writeSpeed(wheel_speed)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass





