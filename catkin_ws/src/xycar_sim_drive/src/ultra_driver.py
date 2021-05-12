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


stackTurn = 0
stackStraightL = 0
stackStraightR = 0
angleAccel = 1
while not rospy.is_shutdown():
    angle = 0  
    if xycar_data.data:
        if xycar_data.data[1] < 300:
            stackStraightL = 0
            stackStraightR = 0
            stackTurn+=1
            if xycar_data.data[0]>xycar_data.data[2]:
                angle = angle - angleAccel * stackTurn^2
            else:
                angle = angle + angleAccel * stackTurn^2
        else:
            stackTurn = 0
            if xycar_data.data[7]>xycar_data.data[6] + 10:
                angle = angle - angleAccel * stackStraightL^2
                stackStraightL+=1
                stackStraightR = 0
                
            elif xycar_data.data[7]<xycar_data.data[6] - 10:
                angle = angle + angleAccel * stackStraightR^2
                stackStraightR+=1
                stackStraightL = 0
            else:
                angle = 0
    xycar_msg.data = [angle, 50]
    motor_pub.publish(xycar_msg)
