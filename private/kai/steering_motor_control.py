#!/usr/bin/env python

import rospy
import time
import orienbus

from geometry_msgs.msg import Twist

kp = 10
kd = 0.1
ki = 0.01

MAX_SPEED = 500

motors = {"lf": lf_angle, "lb": lb_angle, "rf":rf_angle, "rb": rb_angle, "m_time": rospy.get_rostime(), "prev_time": rospy.get_rostime()}
targets = {"lf": des_lf, "lb": des_lb, "rf": des_rf, "rb": des_rb}

port = '/dev/ttyUSB0' # modbus port

# Create orienbus object with port name
orienbus = orienbus.OrienBus(port)

# slave addresses
address_rot_lb = 13
address_rot_rb = 12
address_rot_lf = 11
address_rot_rf = 10

prev_time = 0

period = 0.1

class Motor():
    def __init__(self, address, name):
        self.motor = orienbus.initialize(address)
        self.name = name
        self.curr_err = 0
        self.prev_err = 0
        self.accu_err = 0

    def adjust_speed(self):
        self.curr_error = targets[self.name] - motors[self.name]
        self.accu_err += self.curr_err
        p = proportional(targets[self.name], motors[self.name])
        d = derivative(self.curr_err, self.prev_err)
        i = integral(self.accu_error)
        speed = p + i + d
        if abs(speed) < abs(MAX_SPEED):
            self.motor.writeSpeed(min(speed, MAX_SPEED))
        else:
            if speed < 0:
                self.motor.writeSpeed(-MAX_SPEED)
            else:
                self.motor.writeSpeed(MAX_SPEED)
        self.prev_err = self.curr_err

def encoder_pos(data): # Twist message, current angles in degrees
    motors["lf"] = data.linear.x
    motors["lb"] = data.linear.y
    motors["rf"] = data.linear.z
    motors["rb"] = data.angular.x
    motors["m_time"] = rospy.get_rostime()


def desired_pos(data): # Reading from a twist message containing the cmd for the traget angle for each unit
    targets["lf"] = data.linear.x
    targets["lb"] = data.linear.y
    targets["rf"] = data.linear.z
    targets["rb"] = data.angular.x


def proportional(desired, actual): #error - current angle
    return kp * (desired - actual)

def derivative(curr, prev): #d angle-error/ dt
    return kd * (curr - prev) / (motors["m_time"] - motors["prev_time"])

def integral(accu): 
    return ki * accu

def controller(motor_ls):
    rospy.init_node('steering_control')
    enc_sub = rospy.Subscriber("encoder_positions", Twist, encoder_pos)
    targets = rospy.Subscriber("target_angle", Twist, desired_pos)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in motor_ls:
            i.adjust_speed()
            motors["prev_time"] = motors["m_time"]
        rate.sleep()
    rospy.spin()

def initialize():
    lf = Motor(address_rot_lf, "lf")
    lb = Motor(address_rot_lb, "lb")
    rf = Motor(address_rot_rf, "rf")
    rb = Motor(address_rot_rb, "rb")
    motor_ls = [lf, lb, rf, rb]

'''
def run_node():
    rospy.init_node('steering_control')
    enc_sub = rospy.Subscriber("encoder_positions", Twist, encoder_pos)
    targets = rospy.Subscriber("target_angle", Twist, desired_pos)
    controller(motor_ls)

    rospy.spin()
'''    

if __name__ == "__main__":
    try:
        initialize()
        controller()


    except rospy.ROSInterruptException:
        pass
