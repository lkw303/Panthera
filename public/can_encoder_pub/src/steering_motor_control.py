#!/usr/bin/env python

import rospy
import time
import orienbus

from geometry_msgs.msg import Twist

kp = 10
kd = 0.1
ki = 0.01

MAX_SPEED = 500

#port = '/dev/ttyUSB0' # modbus port

# Create orienbus object with port name
#orienbus = orienbus.OrienBus(port)

# slave addresses
address_rot_lb = 13
address_rot_rb = 12
address_rot_lf = 11
address_rot_rf = 10

prev_time = 0

period = 0.1

m_time = 0
prev_time = 0

lf = 0
lb = 0
rf = 0
rb = 0

t_lf = 0
t_lb = 0
t_rf = 0
t_rb = 0

class Motor():
    def __init__(self, name):
        #self.motor = orienbus.initialize(address)
        self.name = name
        self.curr_err = 0
        self.prev_err = 0
        self.accu_err = 0

    def adjust_speed(self):
    	global t_rf
    	global rf
        #self.curr_error = targets[self.name] - motors[self.name]
        self.curr_err = t_rf - rf
        self.accu_err += self.curr_err
        p = proportional(t_rf, rf)
        d = derivative(self.curr_err, self.prev_err)
        i = integral(self.accu)
        speed = p + i + d
        if abs(speed) < abs(MAX_SPEED):
            #self.motor.writeSpeed(min(speed, MAX_SPEED))
            print(min(speed, MAX_SPEED))
        else:
            if speed < 0:
                #self.motor.writeSpeed(-MAX_SPEED)
                print(-MAX_SPEED)
            else:
                #self.motor.writeSpeed(MAX_SPEED)
                print(MAX_SPEED)
        self.prev_err = self.curr_err

def encoder_pos(data): # Twist message, angles in degrees
	global rf
	global m_time
	lf = data.linear.x
	lb = data.linear.y
	rf = data.linear.z
	rb = data.angular.x
	m_time = rospy.get_rostime()

def desired_pos(data): # Reading from cmd_vel twist message
	global t_rf
	t_lf = data.linear.x
	t_lb = data.linear.y
	t_rf = data.linear.z
	t_rb = data.angular.x
	
def proportional(desired, actual):
    return kp * (desired - actual)

def derivative(curr, prev):
    return kd * (curr - prev) / (m_time - prev_time)

def integral(accu):
    return ki * accu

def controller(motor_ls):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in motor_ls:
            i.adjust_speed()
            print(motors["rxf"])
            prev_time = m_time
        rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("steering_control")
        motors = {"lf": 0, "lb":0, "rf":0, "rb": 0}
        targets = {"lf": 0, "lb": 0, "rf": 0, "rb": 0}
        enc_sub = rospy.Subscriber("encoder_positions", Twist, encoder_pos)
        #targets = rospy.Subscriber("target_angles", Twist, desired_pos)
        lf = Motor("lf")
        lb = Motor("lb")
        rf = Motor("rf")
        rb = Motor("rb")
        motor_ls = [rf]
        controller(motor_ls)

    except rospy.ROSInterruptException:
        pass
