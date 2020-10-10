#!/usr/bin/env python3

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




def key_command_pub():
    global state
    global target
    
    pub = rospy.Publisher("target_angle", Twist, queue_size =10 )
    rospy.init_node('target_angle_pub', anonymous= True)

    rate = rospy.Rate(10)

    #rf =1 lf =2 rb =3 lb =4

    #0,1,2

    while not rospy.is_shutdown():
        char = None
        try:
            char = getch.getch()
        except:
            pass

        if char == '0':
            target = Twist()
            target.linear.x =      0
            target.linear.y =      0 
            target.linear.z =      0
            target.angular.x =     0
            pub.publish(target)

        elif char == '1':
            target = Twist()
            target.linear.x =      -90
            target.linear.y =      -90 
            target.linear.z =      -90
            target.angular.x =     -90
            pub.publish(target)

        elif char == '2':
            target = Twist()
            target.linear.x =      90
            target.linear.y =      90 
            target.linear.z =      90
            target.angular.x =     90
            pub.publish(target)
        
        else:
            pass

            
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        key_command_pub()
    
    except rospy.ROSInterruptException:
        pass
