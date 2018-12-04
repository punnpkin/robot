# coding=utf-8

import rospy
from movement import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist

g_linear_scale = 1.0
g_angular_scale = 1.0

def key_trans(msg):
    if len(msg.data) == 0 :
        return

    if msg.data[0] == 'w':
        t_up(20 * g_linear_scale, 1)

    elif msg.data[0] == 'x':
        t_down(20 * g_linear_scale, 1)

    elif msg.data[0] == 'a':
        t_left(20 * g_angular_scale, 1)

    elif msg.data[0] == 'd':
        t_right(20 * g_angular_scale, 1)

    elif msg.data[0] == 's':
        t_stop(1)



if __name__ == '__main__':
    rospy.init_node('keys_to_move')

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