#!usr/bin/env python
# -*- coding: utf-8 -*-
# 

import rospy
from Servo import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist

g_linear_scale = 1.0
g_angular_scale = 1.0

def key_trans(msg):
    if len(msg.data) == 0 :
        return

    elif msg.data[0] == 'w':
        t_up(20 * g_linear_scale,0)    #前进
    elif msg.data[0] == 'x':
        t_down(20 * g_linear_scale,0)  #后退
    elif msg.data[0] == 'd':
        t_right(20 * g_angular_scale,0) #右转
    elif msg.data[0] == 'a':
        t_left(20 * g_angular_scale,0)  #左转
    elif msg.data[0] == 's':
        t_stop(0)     #停止
    elif msg.data[0] == '0':
        
        Servo_stop()
        time.sleep(ServoDelayTime)
    elif msg.data[0] =='n':  #1
        
        BottomLeft()
        time.sleep(ServoDelayTime)
    elif msg.data[0] =='m':  #2
        
        BottomRight()
        time.sleep(ServoDelayTime)
    elif msg.data[0] =='h':  #4
        
        Arm_A_Up()
        time.sleep(ServoDelayTime)
    elif msg.data[0] =='j':  #上
        
        Arm_A_Down()
        time.sleep(ServoDelayTime)         
    elif msg.data[0] =='y':   #左
        
        Arm_B_Up()
        time.sleep(ServoDelayTime) 
    elif msg.data[0] =='u':    #下
        
        Arm_B_Down()
        time.sleep(ServoDelayTime)            
    elif msg.data[0] =='i':  #打开手爪
        
        ClampOpen()
    elif msg.data[0] =='o':  #闭合手爪
        
        ClampClose()



if __name__ == '__main__':
    rospy.init_node('keys_to_move')
    try:  
        rospy.Subscriber('keys', String, key_trans)

        if rospy.has_param('~linear_scale'):
            g_key_scale = rospy.get_param('~linear_scale')
        else:
            rospy.logwarn("scale is not provided; using %.1f" % g_linear_scale)

        if rospy.has_param('~angular_scale'):
            g_key_scale = rospy.get_param('~angular_scale')
        else:
            rospy.logwarn("scale is not provided; using %.1f" % g_angular_scale)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    finally:
        GPIO.cleanup()