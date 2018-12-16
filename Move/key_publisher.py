#!/usr/bin/env python
#-*- coding: utf-8 -*-
#主要功能：发布 key主题，将用户按键-回车-读取模式转为按键-读取模式
#


import sys, tty, select, termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    key_pub = rospy.Publisher('key', String, queue_size = 1)
    rospy.init_node('keyboard_driver')
    rate = rospy.Rate(100)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print ('Publishing keystrokes...')

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
