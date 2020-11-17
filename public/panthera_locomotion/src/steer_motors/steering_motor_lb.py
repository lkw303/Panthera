#!/usr/bin/env python

'''
Things to note for each motor node:
1. Address #
2. Port /dev/ttyUSB* if using multiple USB ports
3. PID parameters
4. Subscribing to correct twist message for both subscribers
5. Node name
'''

import rospy
import time
import orienbus
import serial.tools.list_ports

from geometry_msgs.msg import Twist
from panthera_locomotion.srv import Status, StatusResponse

# PID parameters
kp = rospy.get_param('lb_pid')['kp']
ki = rospy.get_param('lb_pid')['ki']
kd = rospy.get_param('lb_pid')['kd']

MAX_SPEED = rospy.get_param('/lb_max_speed')
motor_speed = 0

######################
address = 13
sn = rospy.get_param('/lb_serial_number')
p = list(serial.tools.list_ports.grep(sn))
port = '/dev/' + p[0].name
######################

tolerance = rospy.get_param('/angle_tolerance') # Tolerance before moving

target = 0     # target motor position
position = 0   # current motor position

# Errors for pid controller
current_error = 0
prev_error = 0
accu_error = 0
integral_reset = rospy.get_param('/lb_pid')['ir']

orienbus = orienbus.OrienBus(port)
motor = orienbus.initialize(address)


### Subscriber Functions ###
######################################
# Reading actual encoder position
def encoder_pos(data):
    global position, current_time
    position = data.linear.x # take note using correct Twist msg data
    #time = rospy.get_rostime()
    #current_time = time.secs + time.nsecs*(10**(-9))

# Reading target position
def desired_pos(data):
    global target
    target = data.linear.x #take note using correct Twist msg data
######################################

### PID Controller Functions ###
######################################
def proportional(desired, actual): #error - current angle
    prop =  kp * (desired - actual)
    #print("Kp: " + str(prop))
    return prop

def derivative(curr, prev, dt): #d angle-error/ dt
    #dt = 0.05 #current_time - prev_time
    #print("dt is: " + str(dt))
    if dt == 0:
        deriv = 0
    else:
        deriv = kd * (curr - prev) / dt
    #print("Kd: "+ str(deriv))
    return deriv

def integral(accu):
    integral =  ki * accu
    #print("Ki: " +str(integral))
    return integral
######################################

# Adjust motor speed
def adjust_speed(t):
    global motor_speed
    global current_error
    global prev_error
    global accu_error
    global prev_time
    global integral_reset
    current_error = target - position
    #print("current error is:" + str(current_error))
    accu_error += current_error
    p = proportional(target, position)
    d = derivative(current_error, prev_error,t)
    i = integral(accu_error)
    speed = p + i + d
    speed =  -int(speed)
    #print("input speed is: " + str(speed))
    if abs(speed) < abs(MAX_SPEED):
        motor_speed = speed
    	motor.writeSpeed(speed)
    	#print("Speed: "+str(speed))
    else:
        if speed < 0:
            motor_speed = -MAX_SPEED
            motor.writeSpeed(-MAX_SPEED)
            #print("Speed: "+str(-MAX_SPEED))
        else:
            motor_speed = MAX_SPEED
            motor.writeSpeed(MAX_SPEED)
            #print("Speed: "+str(MAX_SPEED))
    prev_error = current_error
    #prev_time = current_time
    #print("accu_error: " + str(accu_error))
    
    if abs(i) >= integral_reset:
    	accu_error = 0

def callback(req):
    x = False
    if abs(current_error) <= tolerance and abs(motor_speed) < 80:
        x = True
    else:
        x = False
    #print(x)
    return StatusResponse(x)
	
if __name__ == "__main__":
    try:
        rospy.init_node("lb_steering_motor") # Note correct node name
        rospy.Subscriber("encoder_positions", Twist, encoder_pos)
        rospy.Subscriber("target_angle", Twist, desired_pos)
        service = rospy.Service('lb_steer_status', Status, callback) # Note correct service

        period = 0.05
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if abs(position) >= 95:
                motor.writeSpeed(0)
                break
            else:
                start = rospy.get_time()
                adjust_speed(period)
                rate.sleep()
                end = rospy.get_time()
                period = end - start
    except rospy.ROSInterruptException:
        pass
