#!/usr/bin/env python

import rospy
import orienbus
import time
import keyboard
import os

from std_msgs.msg import Int8

def key_command_pub():
    pub = rospy.Publisher("key_commands", Int8, queue_size =10 )
    rospy.init_node('key_command_pub', anonymous= True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if keyboard.is_pressed('9'):
            pub.publish(9)
        
        elif keyboard.is_pressed('0'):
            pub.publish(0)

        elif keyboard.is_pressed('1'):
            pub.publish(1)

        elif keyboard.is_pressed('2'):
            pub.publish(2)
        
        else:
            pub.publish(-1)

if __name__ == '__main__':
    try:
        run_sudo()
        key_command_pub()
    
    except rospy.ROSInterruptException:
        pass
