#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : main_drive.py
# 작 성 자 : 신홍재, 황예원
# 생 성 일 : 2021년 07월 10일
# 수 정 일 : 2021년 07월 13일
####################################################################

import time
import rospy, rospkg
import numpy as np
import cv2
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from detect_line import processImage
from driving_method import getSteerAng
from obstacle_detect import obstacle

import sys
import os
import signal

# Variable Initailization

Width = 640
Height = 480

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None

def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def setAnglenSpeed(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

def startDrive():
    global pub
    global image
    global cap
    global Width, Height

    rospy.init_node('main_control')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.sleep(2)
    driving_status = 0          #(0 :  StandardDriving, 1 : Obstacle, 2 : Parking, 3 : Crosswalk)
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    out = cv2.VideoWriter('/home/nvidia/test/test.avi', fourcc, 30.0, (640, 480))


    while True:
        while not image.size == (640*480*3):
            continue

        lpos, rpos = processImage(image)
        obstacle.obstacleDetect()
        out.write(image)

        if driving_status == 0:
            angle = getSteerAng((lpos,rpos))
            speed = 20 
        elif driving_status == 1:
            angle, speed = obstacle.obstacleSteering()
        setAnglenSpeed(angle, speed)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    out.release()
    rospy.spin()
    

if __name__ == '__main__':

    startDrive()




