#!/usr/bin/env python3

import rospy
import time
import can

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

#dic_init = {}

def initialise(dic):

    bustype = "socketcan"
    can_interface = "can0"

    bus = can.interface.Bus(can_interface, bustype = bustype)

    out_1 = bus.recv(1)
    out_2 = bus.recv(1)
    out_3 = bus.recv(1)
    out_4 = bus.recv(1)

        
        
    ls_out = [out_1, out_2, out_3, out_4]
    ls_id = [385, 386, 387, 388]
    dic = {}

    for i in ls_id:
        dic[i] = -1
        
    for i in ls_out:
        for j in ls_id:
            if i.arbitration_id == j:
                position = i.data[:4]
                position_value = int.from_bytes(position, byteorder = "little", signed = False)
                dic[j] = position_value

    return dic
        

def encode_pub(dic):
    pub = rospy.Publisher("encoder_positions" , Twist, queue_size = 10) #topic name
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
        dic_pos = {}

        for i in ls_id:
            dic_pos[i] = -1
        
        for i in ls_out:
            for j in ls_id:
                if i.arbitration_id == j:
                    position = i.data[:4]
                    position_value = int.from_bytes(position, byteorder = "little", signed = False)
                    dic_pos[j] = position_value
        
        
        encoder_pos.linear.x = (dic_pos[385]-dic[385])*0.02197         #/16384x 2 x 180
        encoder_pos.linear.y = (dic_pos[386]-dic[386])*0.02197 
        encoder_pos.linear.z = (dic_pos[387]-dic[387])*0.02197
        encoder_pos.angular.x = (dic_pos[388]-dic[388])*0.02197
        
        '''
        data_1 = out_1.data
        position = data[:4]
        speed = data[4:]

        position_value = int.from_bytes(position,byteorder = 'little' , signed = False)
        speed_value = int.from_bytes(speed, byteorder = 'little' , signed = False)

        rospy.loginfo(position_value)
        pub.publish(position_value)
        '''

        rospy.loginfo(encoder_pos)
        pub.publish(encoder_pos)
if __name__ == '__main__':
    try:
        dic_init = {}
        dic_init = initialise(dic_init)
        encode_pub(dic_init)

    except rospy.ROSInterruptException:
        pass

