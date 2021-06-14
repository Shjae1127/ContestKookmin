#!/usr/bin/env python

import rospy, math
from std_msgs.msg import Int32MultiArray
<<<<<<< HEAD

def callback(msg):
    print(msg.data)

rospy.init_node('guide')
=======
 
    
def callback(msg):
   print(msg.data)
   xycar_data.data = msg.data

rospy.init_node('guide')
xycar_data = Int32MultiArray()
>>>>>>> Assignment_2
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

xycar_msg = Int32MultiArray()

while not rospy.is_shutdown():
    angle = 0
<<<<<<< HEAD
    xycar_msg.data = [angle, 10]
=======
    if xycar_data.data:
        if xycar_data.data[1] < 300:
            if xycar_data.data[0]>xycar_data.data[2]:
                angle = angle - 45
            else:
                angle = angle + 45
        else:
            if xycar_data.data[7]>xycar_data.data[6]+10:
                angle = angle - 40
            elif xycar_data.data[7]<xycar_data.data[6]-10:
                angle = angle+40
            else:
                angle = 0
    xycar_msg.data = [angle, 50]
>>>>>>> Assignment_2
    motor_pub.publish(xycar_msg)
