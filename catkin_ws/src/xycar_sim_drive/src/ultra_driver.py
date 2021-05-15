#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int32MultiArray


def callback(msg):
    print(msg.data)
    distance.data = msg.data


rospy.init_node('guide')
distance = Int32MultiArray()
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

xycar_msg = Int32MultiArray()
angle = 0
while not rospy.is_shutdown():
    if (distance.data):
        fl = distance.data[0]
        fm = distance.data[1]
        fr = distance.data[2]
        l = distance.data[7]
        r = distance.data[6]
        theta = math.radians(36.7)
        ld_x = (fr-fl)*math.cos(theta)/2
        ld_y = (fr+fl)*math.sin(theta)/2
        ld = math.sqrt(pow(ld_x, 2)+pow(ld_y, 2))
        alpha = math.atan(ld_x/ld_y)
        L = 450/3.33
        angle = math.atan((2*L*math.sin(alpha))/ld)
        angle = math.degrees(angle)
    xycar_msg.data = [angle, 15]
    motor_pub.publish(xycar_msg)
