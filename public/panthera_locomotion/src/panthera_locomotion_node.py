#!/usr/bin/env python

import rospy
import orienbus
import time


from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

global state 
state = 0

class unit:
    def __init__(self, address_rot, address_trans):
        self.port = '/dev/ttyUSB0'
        modbus = orienbus.OrienBus(self.port)
        self.address_rot = address_rot
        self.address_trans = address_trans
        self.motor_rot = modbus.initialize(address_rot)
        self.motor_trans = modbus.initialize(address_trans)
        self.angle = 0
        self.in_pos_rot = True
        self.in_pos_trans = True
        self.running_rot = False
        self.running_trans = False
    
    def set_angle(self, angle):
        self.angle = angle
    
    def get_angle(self):
        return self.angle

    def run_rot(self, speed):
        self.motor_rot.writeSpeed(speed)
        self.running_rot = True
        self.in_pos_rot = False

    def run_trans(self, speed):
        self.motor_trans.writeSpeed(speed)
        self.running_trans = True
        self.in_pos_trans = False

    
    def stop_all(self):
        self.motor_rot.writeSpeed(0)
        self.motor_trans.writeSpeed(0)
        self.running_rot = False
        self.running_trans = False

    def stop_rot(self):
        self.motor_rot.writeSpeed(0)
        self.running_rot = False
    
    def stop_trans(self):
        self.motor_rot.writeSpeed(0)
        self.running_trans = False


    

def initialise_units():
    global lf, lb, rf ,rb
    lb = unit(11, 2)
    rb = unit(12, 3)
    lf = unit(11, 2)
    rf = unit(10, 1)

    global ls_left, ls_right, ls_back, ls_front, ls_units
    
    #gotta make these references pointers to ensure that we dont create a few copies of them
    ls_left = [lb, lf]
    ls_right = [rb, rf]
    
    ls_front = [rf, lf]
    ls_back = [rb, lb]

    ls_units = ls_front +ls_back
    
 


def cb(twist_msg):
    global twist
    twist = twist_msg
    print(twist)
    global lf, lb, rf ,rb

    rf.angle = twist_msg.linear.x
    lf.angle = twist_msg.linear.y
    rb.angle = twist_msg.linear.z
    lb.angle = twist_msg.angular.x
    


#states 0,1,2 state 0 >> wheels straight/forward, 1 >> wheels turned clockwise 90 , 2 wheels turned anti-clockwise 90
# unint commands 0,1,2,3,4 command 0 >>> turn back to straight, 1 >>> turn to pos 1 clockwise 90, 2>> turn to pos 2 anticlockwise 90 

def cb1(uint_msg):
    global twist
    global state
    global ls_left, ls_right, ls_back, ls_front, ls_units
    
    print(uint_msg)

    if uint_msg == 9: # emergency stop
        for i in ls_units:
            i.stop_all()
            return "stop all motor"

    elif uint_msg == -1:
        return None

    elif uint_msg == None:
        return "no command"

    elif state == 0:
        if uint_msg ==0:
            return "already at desired position"

        elif uint_msg == 1:
            for i in ls_units:
                i.run_rot(100)
            state = -1

        elif uint_msg == 2 :
            for i in ls_units:
                i.run_rot(-100)
            state = -2


    elif state > 0 :

        if uint_msg == 1:
            if state == 1:
                return "already at desired state"
                
            elif state == 2:
                for i in ls_units:
                    i.writeSpeed(100)
                state = -1
            

        elif uint_msg == 2:
            if state == 2:
                return "already in desired state"

            elif state == 1:
                for i in ls_units:
                    i.running_rot(-100)
                state = -2

        elif uint_msg == 0:
            if state ==1:
                for i in ls_units:
                    i.running_rot(-100)
                state = -9

            elif state == 2:
                for i in ls_units:
                    i.running_rot(100)
                state = -9
            
            
        
    elif state < 0:
        if state == -1:
            ls = []
            for i in ls_units:
                if i.running_rot:
                    if -88> i.angle >- 95:
                        i.stop_rot()
                        i.in_pos_rot = True

                ls.append(i.in_pos_rot)

            if all(ls): #check if all stop runnning
                state = 1
  

        elif state == -2:
            ls = []
            for i in ls_units:
                
                if i.running_rot:
                    if 88< i.angle < 95:
                        i.stop_rot()
                        i.in_pos_rot = True
                        
                ls.append(i.in_pos_rot)
    
            if all(ls):
                state = 2

        elif state == -9:
            ls = []
            for i in ls_units:
                if i.running_rot:
                    if -5 < i.angle< 5:
                        i.stop_rot()
                        i.in_pos_rot = True

                ls.append(i.in_pos_rot)
            if all(ls):
                state = 0
                
    


def listen():
    print("listening")
    rospy.init_node("panthera_locomotion_node", anonymous = True)
    rospy.Subscriber("key_commands", Int8, cb1)
    rospy.Subscriber("encoder_positions", Twist, cb )
    

    rospy.spin()


if __name__ == '__main__':
    initialise_units()
    listen()

