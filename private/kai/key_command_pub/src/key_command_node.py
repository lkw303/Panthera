#!/usr/bin/env python

import rospy
import orienbus
import time
import keyboard
import os
import getch

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

global state
global target

def set_state(inp):
    global state
    if inp == "0".encode("ASCII"):
        state = 0

    elif inp == "1".encode("ASCII"):
        state = 1

    elif inp == "2".encode("ASCII"):
        state = 2


def key_command_pub():
    global state
    global target
    
    pub = rospy.Publisher("target_angle", Int8, queue_size =10 )
    rospy.init_node('target_angle_pub', anonymous= True)

    rate = rospy.Rate(10)

    #rf =1 lf =2 rb =3 lb =4

    #0,1,2

    while not rospy.is_shutdown():
        char = getch.getch()

        set_state(char)

        if state == 0:
            target = Twist()
            target.linear.x =      0
            target.linear.y =      0 
            target.linear.z =      0
            target.angular.x =     0

        if state == 1:
            target = Twist()
            target.linear.x =      -90
            target.linear.y =      -90 
            target.linear.z =      -90
            target.angular.x =     -90

        if state == 2:
            target = Twist()
            target.linear.x =      90
            target.linear.y =      90 
            target.linear.z =      90
            target.angular.x =     90


        pub.publish(target)

if __name__ == '__main__':
    try:
        key_command_pub()
    
    except rospy.ROSInterruptException:
        pass
