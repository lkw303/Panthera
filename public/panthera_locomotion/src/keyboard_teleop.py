#!/usr/bin/env python

import math
import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from panthera_locomotion.srv import Status, StatusRequest, StatusResponse
import serial.tools.list_ports as lp

width = 1.02
length = 1.31

lb = 0
lf = 0
rb = 0
rf = 0

pos_lb = 0
pos_rb = 0
pos_lf = 0
pos_rf = 0

linear_x = 0.3
angular_z = 0.5

tolerance = 1.0

# Bluetooth port
p = list(lp.grep('AQ00F1QY'))
port = '/dev/' + p[0].name

def adjust_wheels(vx, wz): # radius in m, direction c(-1) or ccw(1)
	global lb,lf,rb,rf
	if wz == 0:
		radius = float('inf')
	else:
		radius = vx/wz
	left = radius - width/2
	right = radius + width/2
	
	lf = round(math.degrees(math.atan((length*0.5) / left)), 2)
	rf = round(math.degrees(math.atan((length*0.5) / right)), 2)
	lb = -lf
	rb = -rf

def enc_pos(data):
	global pos_lb, pos_lf, pos_rb, pos_rf
	pos_lb = data.linear.x
	pos_rb = data.linear.y
	pos_lf = data.linear.z
	pos_rf = data.angular.x

def check():
	'''
	global lb, pos_lb
	check = True
	twist = Twist()
	while check:
		if abs(lb - pos_lb) >= tolerance and abs(lf - pos_lf) >= tolerance and abs(rb - pos_rb) >= tolerance and abs(rf - pos_rf) >= tolerance:
		#if abs(rb - pos_rb) >= tolerance:
			print("Adjusting\n")
			print([abs(lb - pos_lb) <= tolerance, abs(rb - pos_rb) <= tolerance, abs(lf - pos_lf) <= tolerance, abs(rf - pos_rf) <= tolerance])
		else:
			check = False
			print('Adjusted')
			print([abs(lb - pos_lb) <= tolerance, abs(rb - pos_rb) <= tolerance, abs(lf - pos_lf) <= tolerance, abs(rf - pos_rf) <= tolerance])
	'''
	req = StatusRequest()
	signal = False
	while signal  == False and not rospy.is_shutdown():
		rospy.sleep(0.5)
		lb = lb_status(req)
		rb = rb_status(req)
		lf = lf_status(req)
		rf = rf_status(req)
		signal = (lb.status and rb.status and lf.status and rf.status)
		print("Status of steering motors:" + str(signal))


def run(x):
	global linear_x
	global angular_z

	twist = Twist()
	twist2 = Twist()

	### MOVEMENT ###

	if x == 'w':
		adjust_wheels(linear_x, 0)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = 0
		pub.publish(twist)

	elif x == 'e':
		adjust_wheels(linear_x, -angular_z)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = -angular_z
		pub.publish(twist)

	elif x == 'q':
		adjust_wheels(linear_x, angular_z)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = angular_z
		pub.publish(twist)

	elif x == 'd':
		adjust_wheels(0, -angular_z)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = 0
		twist.angular.z = -angular_z
		pub.publish(twist)

	elif x == 'a':
		adjust_wheels(0, angular_z)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = 0
		twist.angular.z = angular_z
		pub.publish(twist)

	elif x == 's':
		adjust_wheels(0, 0)
		twist.linear.x = lb
		twist.linear.y = rb
		twist.linear.z = lf
		twist.angular.x = rf
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)

	elif x == 'c':
		adjust_wheels(0, 0)
		twist.linear.x = -90
		twist.linear.y = -90
		twist.linear.z = -90
		twist.angular.x = -90
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = 0
		pub.publish(twist)

	elif x == 'z':
		adjust_wheels(0, 0)
		twist.linear.x = 90
		twist.linear.y = 90
		twist.linear.z = 90
		twist.angular.x = 90
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = linear_x
		twist.angular.z = 0
		pub.publish(twist)

	elif x == 'r':
		adjust_wheels(0, 0)
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist.angular.y = -linear_x
		twist.angular.z = 0
		pub.publish(twist)

	# RECONFIGURATION EXPANSION

	elif x == 't': # Expand left
		adjust_wheels(0, 0)
		twist.linear.x = 90
		twist.linear.y = 0
		twist.linear.z = 90
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = linear_x
		twist2.linear.z = linear_x
		recon.publish(twist2)

		rospy.sleep(2) # Adjust time for expansion length

		twist2.linear.x = 0
		twist2.linear.z = 0
		recon.publish(twist2)

	elif x == 'u': # Expand right
		adjust_wheels(0, 0)
		twist.linear.x = 0
		twist.linear.y = -90
		twist.linear.z = 0
		twist.angular.x = -90
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.y = linear_x
		twist2.angular.x = linear_x
		recon.publish(twist2)

		rospy.sleep(2) # Adjust time for expansion length

		twist2.linear.y = 0
		twist2.angular.x = 0
		recon.publish(twist2)

	elif x == 'y': # Expand both sides
		adjust_wheels(0, 0)
		twist.linear.x = 90
		twist.linear.y = -90
		twist.linear.z = 90
		twist.angular.x = -90
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = linear_x
		twist2.linear.y = linear_x
		twist2.linear.z = linear_x
		twist2.angular.x = linear_x
		recon.publish(twist2)

		rospy.sleep(2) # Adjust time for expansion length

		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)

	# RECONFIGURATION CONTRACTION

	elif x == 'g': # Contract left
		adjust_wheels(0, 0)
		twist.linear.x = -90
		twist.linear.y = 0
		twist.linear.z = -90
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = linear_x
		twist2.linear.z = linear_x
		recon.publish(twist2)

		rospy.sleep(2) # Adjust time for expansion length

		twist2.linear.x = 0
		twist2.linear.z = 0
		recon.publish(twist2)

	elif x == 'j': # Contract right
		adjust_wheels(0, 0)
		twist.linear.x = 0
		twist.linear.y = 90
		twist.linear.z = 0
		twist.angular.x = 90
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.y = linear_x
		twist2.angular.x = linear_x
		recon.publish(twist2)

		rospy.sleep(2) # Adjust time for expansion length

		twist2.linear.y = 0
		twist2.angular.x = 0
		recon.publish(twist2)

	elif x == 'h': # Contract both sides
		adjust_wheels(0, 0)
		twist.linear.x = -90
		twist.linear.y = 90
		twist.linear.z = -90
		twist.angular.x = 90
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
		
		# Check if wheels have adjusted before moving
		check()

		twist2.linear.x = linear_x
		twist2.linear.y = linear_x
		twist2.linear.z = linear_x
		twist2.angular.x = linear_x
		recon.publish(twist2)

		rospy.sleep(2) # Adjust time for expansion length

		twist2.linear.x = 0
		twist2.linear.y = 0
		twist2.linear.z = 0
		twist2.angular.x = 0
		recon.publish(twist2)

	# ADJUST VX AND WZ DEFAULT SPEEDS

	elif x == 'o':
		linear_x += 0.05

	elif x == 'p':
		angular_z += 0.05

	elif x == 'n':
		linear_x -= 0.05

	elif x == 'm':
		angular_z -= 0.05
		
	print("\nvx: " + str(linear_x) + "m/s")
	print("wz: " + str(angular_z) + "rad/s")
	print("Turning Radius: " + str(linear_x/angular_z) + "m\n")

if __name__ == "__main__":
	try:
		rospy.init_node('controller')
		ser = serial.Serial(port)
		sub = rospy.Subscriber('encoder_positions', Twist, enc_pos)
		pub = rospy.Publisher('target_angle', Twist, queue_size=1)
		recon = rospy.Publisher('reconfig', Twist, queue_size=1)

		rospy.wait_for_service('/lb_steer_status')
		rospy.wait_for_service('/lf_steer_status')
		rospy.wait_for_service('/rb_steer_status')
		rospy.wait_for_service('/rf_steer_status')

		lb_status = rospy.ServiceProxy('lb_steer_status', Status)
		lf_status = rospy.ServiceProxy('lf_steer_status', Status)
		rb_status = rospy.ServiceProxy('rb_steer_status', Status)
		rf_status = rospy.ServiceProxy('rf_steer_status', Status)

		switch = True
		while switch:
			print("w: Forward")
			print("r: Reverse")
			print("e: Turn Right with a radius vx/wz")
			print("q: Turn Left with a radius vx/wz")
			print("d/a: L/R On the spot rotation")
			print("s: Align wheels to 0 and stop")
			print("o/p: increase vx and wz by +0.05")
			print("n/m: decrease vx and wz by -0.05")
			print("c: 90 Degrees right")
			print("z: 90 Degrees left")
			print("t: Expand Left")
			print("u: Expand Right")
			print("y: Expand Both Sides")
			print("g: Contract Left")
			print("j: Contract Right")
			print("h: Contract Both Sides")
			cmd = ser.readline().strip('\r\n')
			print("\n" + cmd)
			if cmd == ' ':
				switch = False
			else:
				run(cmd)
	except rospy.ROSInterruptException:
		pass


