#!/usr/bin/env python

import rospy
import time
import orienbus

from geometry_msgs.msg import Twist

kp = 6
kd = 0.5
ki = 0.2

MAX_SPEED = 500

motors = {"lf": 0, "lb": 0, "rf":0, "rb": 0, "m_time": 0, "prev_time": 0}
targets = {"lf": 0, "lb": 0, "rf": 0, "rb": 0}

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

global motor_ls

class Motor():
    def __init__(self, address, name):
        self.motor = orienbus.initialize(address)
        self.name = name
        self.curr_err = 0
        self.prev_err = 0
        self.accu_err = 0

    def adjust_speed(self):
        self.curr_error = targets[self.name] - motors[self.name]
        print("current error is:" +str(self.curr_error))
        self.accu_err += self.curr_err
        p = proportional(targets[self.name], motors[self.name])
        d = derivative(self.curr_err, self.prev_err)
        i = integral(self.accu_err)
        speed = p + i + d
        speed =  -int(speed)
        print("input speed is: " + str(speed))
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
    time = rospy.get_rostime()
    motors["m_time"] = time.secs + time.nsecs*(10**(-9))


def desired_pos(data): # Reading from a twist message containing the cmd for the traget angle for each unit
    targets["lf"] = data.linear.x
    targets["lb"] = data.linear.y
    targets["rf"] = data.linear.z
    targets["rb"] = data.angular.x


def proportional(desired, actual): #error - current angle
    prop =  kp * (desired - actual)
    print("prop is: " + str(prop))
    return prop

def derivative(curr, prev): #d angle-error/ dt
    dt = (motors["m_time"] - motors["prev_time"])
    print("dt ios: " + str(dt))
    if dt == 0:
        deriv = 0
    else:
        deriv = kd * (curr - prev) / dt
    print("deriv is: "+ str(deriv))
    return deriv

def integral(accu):
    integral =  ki * accu
    print("integral is : " +str(integral))
    return integral

def controller(motor_ls):
    rospy.init_node('steering_control')
    rospy.Subscriber("encoder_positions", Twist, encoder_pos)
    rospy.Subscriber("target_angle", Twist, desired_pos)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
 
        for i in motor_ls:
            i.adjust_speed()
            motors["prev_time"] = motors["m_time"]
            rate.sleep()
    rospy.spin()

def initialize():
    global motor_ls
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
        controller(motor_ls)


    except rospy.ROSInterruptException:
        pass
