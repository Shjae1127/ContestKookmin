#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : main_drive.py
# 작 성 자 : 신홍재
# 생 성 일 : 2021년 07월 10일
# 수 정 일 : 
####################################################################

import rospy, rospkg
import numpy as np
import cv2
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from detect_line import Offset, detectLine
from driving_method import Width, Height, getSteerAng

import sys
import os
import signal

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

# draw rectangle
def drawRectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                  (lpos + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                  (rpos + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img, (center - 5, 15 + offset),
                  (center + 5, 25 + offset),
                  (0, 255, 0), 2)
    cv2.rectangle(img,(320-5,offset+15),(320+5,offset+25),(0,0,255),2)
    return img

# # left lines, right lines
# def divide_left_right(lines):
#     global Width

#     low_slope_threshold = 0
#     high_slope_threshold = 10

#     # calculate slope & filtering with threshold
#     slopes = []
#     new_lines = []

#     for line in lines:
#         x1, y1, x2, y2 = line[0]

#         if x2 - x1 == 0:
#             slope = 0
#         else:
#             slope = float(y2-y1) / float(x2-x1)
        
#         if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
#             slopes.append(slope)
#             new_lines.append(line[0])

#     # divide lines left to right
#     left_lines = []
#     right_lines = []

#     for j in range(len(slopes)):
#         Line = new_lines[j]
#         slope = slopes[j]

#         x1, y1, x2, y2 = Line

#         if (slope < 0) and (x2 < Width/2 - 90):
#             left_lines.append([Line.tolist()])
#         elif (slope > 0) and (x1 > Width/2 + 90):
#             right_lines.append([Line.tolist()])

#     return left_lines, right_lines


# show image and return lpos, rpos
def processImage(frame):
    global Offset
    frame, lpos, rpos = detectLine(frame)
    if lpos < 0:
        lpos = 0
    if rpos >640:
        rpos = 640
    frame = drawRectangle(frame, lpos, rpos, offset=Offset)
    cv2.imshow('image', frame)
    return (lpos, rpos)

def startDrive():
    global pub
    global image
    global cap
    global Width, Height

    rospy.init_node('main_control')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.sleep(2)

    while True:
        while not image.size == (640*480*3):
            continue

        lpos, rpos = processImage(image)

        center = (lpos + rpos) / 2
        angle = (Width/2 - center)
        setAnglenSpeed(angle, 20)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__ == '__main__':

    startDrive()


