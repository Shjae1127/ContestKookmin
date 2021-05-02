#!/usr/bin/env python

import rospy, math
from std_msgs.msg import Int32MultiArray
 
    
def callback(msg):
   print(msg.data)
   xycar_data.data = msg.data

rospy.init_node('guide')
xycar_data = Int32MultiArray()
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

xycar_msg = Int32MultiArray()

while not rospy.is_shutdown():
    angle = 0
    if xycar_data.data:
        if xycar_data.data[1] < 300:
            if xycar_data.data[0]>xycar_data.data[2]:
                angle = angle - 50
            else:
                angle = angle + 50
        else:
            if xycar_data.data[7]>xycar_data.data[6]:
                angle = angle - 50
            else :
                angle = angle+50
    xycar_msg.data = [angle, 50]
    motor_pub.publish(xycar_msg)
