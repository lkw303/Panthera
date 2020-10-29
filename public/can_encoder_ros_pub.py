#!/usr/bin/env python3

import rospy
import time
import can

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

#dic_init = {}

global dic_init
global bus
dic_init = {}

def initialise():
    global dic_init
    global bus
    bustype = "socketcan"
    can_interface = "can0"

    bus = can.interface.Bus(can_interface, bustype = bustype)

    out_1 = bus.recv(1)
    out_2 = bus.recv(1)
    out_3 = bus.recv(1)
    out_4 = bus.recv(1)

        
        
    ls_out = [out_1, out_2, out_3, out_4]
    ls_id = [385, 386, 387, 388]
    #dic = {}

    for i in ls_id:
        dic_init[i] = -1
        
    for i in ls_out:
        for j in ls_id:
            if i.arbitration_id == j:
                position = i.data[:4]
                position_value = int.from_bytes(position, byteorder = "little", signed = False)
                dic_init[j] = position_value

    return dic

def recalib_encoder(data):
    global bus
    global dic_init

    out_1 = bus.recv(1)
    out_2 = bus.recv(1)
    out_3 = bus.recv(1)
    out_4 = bus.recv(1)

        
    ls_out = [out_1, out_2, out_3, out_4]
    ls_id = [385, 386, 387, 388]

    target_encoder = None

    if data == 1:
        target_encode_idx = 0
    if data == 2:
        target_encoder_idx = 1
    if data == 3:
        target_encoder_idx = 2
    if data == 4:
        target_encoder_idx = 3
        
    for i in ls_out:
        if i.arbitration_id == ls_id[target_encode_idx]:
            position = i.data[:4]
            position_value = int.from_bytes(position, byteorder = "little", signed = False)
            dic_init[target_encode_idx] = position_value
    




def encode_pub():
    global dic_init
    pub = rospy.Publisher("encoder_positions" , Twist, queue_size = 10) #topic name
    sub = rospy.Subscriber("recalibrate_encoder", Int, recalib_encoder)
    #twist_pub = rospy.Publisher("encoder_positions", Twist, queue_size = 10) #topic name

    rospy.init_node('encode_pub', anonymous = True) #node name
    rate = rospy.Rate(10)
    
    bustype = "socketcan"
    can_interface = "can0"

    bus = can.interface.Bus(can_interface, bustype = bustype)
    

    while not rospy.is_shutdown():
        out_1 = bus.recv(1)
        out_2 = bus.recv(1)
        out_3 = bus.recv(1)
        out_4 = bus.recv(1)

        
        encoder_pos = Twist()
        
        ls_out = [out_1, out_2, out_3, out_4]
        ls_id = [385, 386, 387, 388]
        #dic_pos = {}

        for i in ls_id:
            dic_pos[i] = -1
        
        for i in ls_out:
            for j in ls_id:
                if i.arbitration_id == j:
                    position = i.data[:4]
                    position_value = int.from_bytes(position, byteorder = "little", signed = False)
                    dic_pos[j] = position_value
        
        
        encoder_pos.linear.x = (dic_pos[385]-dic_init[385])*0.02197         #/16384x 2 x 180
        encoder_pos.linear.y = (dic_pos[386]-dic_init[386])*0.02197 
        encoder_pos.linear.z = (dic_pos[387]-dic_init[387])*0.02197
        encoder_pos.angular.x = (dic_pos[388]-dic_init[388])*0.02197
        

        rospy.loginfo(encoder_pos)
        pub.publish(encoder_pos)
if __name__ == '__main__':
    try:
        #dic_init = {}
        initialise(dic_init)
        encode_pub()

    except rospy.ROSInterruptException:
        pass

