#!/usr/bin/env python

import rospy
import time
import orienbus

from geometry_msgs.msg import Twist
'''
# For motor address 11,10,12
kp = 19
kd = 0.01
ki = 10


# For motor address 13
kp = 6
kd = 0.5
ki = 0.1
'''

MAX_SPEED = 600
global motors
global targets
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
    def __init__(self, address, name, kp, kd, ki):
        self.motor = orienbus.initialize(address)
        self.name = name
        self.angle = 0
        self.target = 0
        self.curr_err = 0
        self.prev_err = 0
        self.accu_err = 0
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.time = 0
        self.prev_time =0

    def adjust_speed(self):
        global motors
        global targets
        self.curr_error = targets[self.name] - motors[self.name]
        print("current error is:" +str(self.curr_error))
        self.accu_err += self.curr_err
        print("accu error is:" +str(self.accu_err))
        print("prev error is:" +str(self.prev_err))
        p = proportional(targets[self.name], motors[self.name], self.kp)
        d = derivative(self.curr_err, self.prev_err, self.kd)
        i = integral(self.accu_err, self.ki)
        speed = p + i + d
        speed =  -int(speed)
        print("input speed is: " + str(speed))
        if abs(speed) < abs(MAX_SPEED):
            self.motor.writeSpeed(speed)
        else:
            if speed < 0:
                self.motor.writeSpeed(-MAX_SPEED)
            else:
                self.motor.writeSpeed(MAX_SPEED)
        self.prev_err = self.curr_err
        

def encoder_pos(data): # Twist message, current angles in degree 
    global motor_ls
    motor_ls[0].angle = data.linear.z
   
    time = rospy.get_rostime()

    motor_ls[0].time = time.secs + time.nsecs*(10**(-9))


def desired_pos(data): # Reading from a twist message containing the cmd for the traget angle for each unit
    global motor_ls
    motor_ls[0].target = data.linear.z


def proportional(desired, actual, kp): #error - current angle
    prop =  kp * (desired - actual)
    print("prop is: " + str(prop))
    return prop

def derivative(curr, prev, kd): #d angle-error/ dt
    
    deriv = (kd * (curr - prev)) / 0.05
    '''
    dt = (motors["m_time"] - motors["prev_time"])
    print("dt is: " + str(dt))
    if dt == 0:
        deriv = 0
    else:
        deriv = kd * (curr - prev) / dt
    print("deriv is: "+ str(deriv))
    return deriv
    '''
    print("deriv is: "+ str(deriv))
    return deriv

def integral(accu, ki):
    integral =  ki * accu
    print("integral is : " +str(integral))
    return integral

def controller(motor_ls):
    rospy.init_node('steering_control')
    rospy.Subscriber("encoder_positions", Twist, encoder_pos)
    rospy.Subscriber("target_angle", Twist, desired_pos)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
 
        for i in motor_ls:
            i.adjust_speed()
	    i.prev_time = i.time
            #motors["prev_time"] = motors["m_time"]
            rate.sleep()
    rospy.spin()

def initialize():
    global motor_ls
    lf = Motor(address_rot_lf, "lf", 20, 50, 20) # address, name, kp, kd ,ki
    lb = Motor(address_rot_lb, "lb", 6, 0.5, 0.1) # Lower gear ratio
    rf = Motor(address_rot_rf, "rf", 19, 0.01, 12)
    rb = Motor(address_rot_rb, "rb", 19, 0.01, 12)
    #motor_ls = [lf, lb, rf, rb]
    motor_ls = [lf,lb]

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
