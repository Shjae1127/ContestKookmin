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
from detect_line import Offset, detectLine
from driving_method import Width, Height, getSteerAng
from obstacle_detect import lidar

import sys
import os
import signal

# Variable Initailization
lpos, rpos = 125, 450
pathwidth = rpos - lpos
Width = 640
Height = 480
Gap = 40
cpos = 266

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

# 중앙 차선 좌표도 그릴 수 있도록 수정
# draw rectangle
def drawRectangle(img, lpos, rpos, cpos, offset=0):
    # center = (lpos + rpos) / 2
    center = cpos

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

# get the rectangle position of a center line
def getRectangle(img):
    _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return 0, 0, 0, 0
    cnt = contours[0]

    x, y, w, h = cv2.boundingRect(cnt)
    # print('x: ', x, 'y: ', y)
    # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 3)

    # cv2.imshow('rec', img)
    return x, y, w, h

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

# Get the positions of left lane, right lane
def detectLine(frame):
    global lpos, rpos, cpos, pathwidth
    #####  Variable Initailization  #####
    src = frame
    w, h = 150, 50
    threshold, minLineLength, maxLineGap = 50, 5, 100
    num_L, num_R = 0, 0
    line_L_X1, line_L_Y1 = 0, 0
    line_L_X2, line_L_Y2 = 0, 0
    line_R_X1, line_R_Y1 = 0, 0
    line_R_X2, line_R_Y2 = 0, 0
    
    ##### Hough Line Transform #####
    frame = cv2.Canny(frame, 400, 200, None, 3)     # detect the edges of image by using Canny detector
    w = 100
    x_L = lpos-w/2
    if x_L < 0:
        x_L = 0
    x_R = rpos-w/2

    ### Set ROI tracing lpos, and rpos  ###
    roi_L = frame[Offset:Offset+h, x_L:x_L+w]       
    roi_R = frame[Offset:Offset+h, x_R:x_R+w]
    roi_C = frame[340:340+Gap, 100:Width-100] # 센터라인용 roi인데 실제로 돌려보고 조정해도 괜츈(roi더 넓게 하되 이전 좌표와의 편차로 벗어나는 값들 거르기)
    linesP_L = cv2.HoughLinesP(
        roi_L, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)      #run Probablistic Hough Line Transform for detect the line of left side
    linesP_R = cv2.HoughLinesP(
        roi_R, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)      #run Probablistic Hough Line Transform for detect the line of right side
    
    # For stability, calculate the average position
    if linesP_L is not None:
        for i in range(0, len(linesP_L)):
            l = linesP_L[i][0]
            line_L_X1 = line_L_X1 + x_L+l[0]
            line_L_Y1 = line_L_Y1 + Offset+l[1]
            line_L_X2 = line_L_X2 + x_L+l[2]
            line_L_Y2 = line_L_Y2 + Offset+l[3]
            num_L = num_L + 1

    if linesP_R is not None:
        for i in range(0, len(linesP_R)):
            l = linesP_R[i][0]
            line_R_X1 = line_R_X1 + x_R+l[0]
            line_R_Y1 = line_R_Y1 + Offset+l[1]
            line_R_X2 = line_R_X2 + x_R+l[2]
            line_R_Y2 = line_R_Y2 + Offset+l[3]
            num_R = num_R + 1

    # For accuracy, update lpos and rpos only if 0 < number of detected lines < 3      
    if (num_L > 0) & (num_L < 3):
        line_L_X1, line_L_Y1 = line_L_X1/num_L,  line_L_Y1/num_L
        line_L_X2, line_L_Y2 = line_L_X2/num_L, line_L_Y2/num_L
        #cv2.line(src, (line_L_X1, line_L_Y1), (line_L_X2,
                                            #    line_L_Y2), (0, 0, 255), 3, cv2.LINE_AA)
        lpos = (Offset + h/2-line_L_Y1)*(line_L_X1-line_L_X2) / \
            (line_L_Y1-line_L_Y2)+line_L_X1

    if (num_R > 0) & (num_R < 3):
        line_R_X1, line_R_Y1 = line_R_X1/num_R,  line_R_Y1/num_R
        line_R_X2, line_R_Y2 = line_R_X2/num_R, line_R_Y2/num_R
        #cv2.line(src, (line_R_X1, line_R_Y1), (line_R_X2,
                                        #    line_R_Y2), (0, 0, 255), 3, cv2.LINE_AA)
        rpos = (Offset + h/2-line_R_Y1)*(line_R_X1-line_R_X2) / \
            (line_R_Y1-line_R_Y2)+line_R_X1

    ##########################################################################################
    # case1 (num_L!=0 & num_R!=0): both left, right line detected  >>  save pathwidth #
    # case2,3 ((num_L!=0 & num_R==0) | (num_L==0 & num_R!=0)): ONLY one side line detected >>  set imaginary line using pathwidth #
    # case4 (num_L==0 & num_R==0): do nothing #
    ##########################################################################################

    if (num_L != 0) & (num_R != 0):
        pathwidth = rpos - lpos
    elif (num_L != 0) & (num_R == 0):
        rpos = lpos + pathwidth
    elif (num_L == 0) & (num_R != 0):
        lpos = rpos - pathwidth


    # Center line
    x, y, w, h = getRectangle(roi_C) # 여기서 받은건 현재 잘라놓은 roi_C에서의 좌표

    # To avoid 엣지 이미지 노이즈때문에 차도에서 관계없는 점 검출됨 and 좌우 차선 부근에서 검출됨 
    if w > 10 and h > 10 and (h+4) >= w: # 여기 사각형 크기나 높이, 너비 비율은 유동적으로 조절해야할듯
        x += 100
        y += Offset
        # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
    
        cpos = x + w/2 # 센터 바운딩박스 중간점
        # frame = draw_rectangle(frame, lpos, rpos, cpos, offset=Offset)

    # else:
    #     frame = draw_rectangle(frame, lpos, rpos, cpos, offset=Offset)

    return src, lpos, rpos, cpos


# show image and return lpos, rpos
def processImage(frame):
    global Offset
    global out
    frame, lpos, rpos, cpos = detectLine(frame) # 센터 좌표 추가
    if lpos < 0:
        lpos = 0
    if rpos >640:
        rpos = 640
    frame = drawRectangle(frame, lpos, rpos, cpos, offset=Offset) # 센터 좌표 추가

    cv2.imshow('image', frame)

    return (lpos, rpos)

# 일단 임시로 주석처리(당장은 ROS사용 X)

def startDrive():
    global pub
    global image
    global cap
    global Width, Height

    rospy.init_node('main_control')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.sleep(2)

    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    out = cv2.VideoWriter('/home/nvidia/test/test.avi', fourcc, 30.0, (640, 480))


    while True:
        while not image.size == (640*480*3):
            continue

        lpos, rpos = processImage(image)
        
        out.write(image)

        center = (lpos + rpos) / 2
        # angle = (Width/2 - center)
        
        angle = getSteerAng((lpos,rpos))
        setAnglenSpeed(angle, 0)
        lidar_range, lidar_angleinc = lidar.getLidarData()
        for i in lidar_range:
            if i < 0.5:
                print(i)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    out.release()
    rospy.spin()
    

if __name__ == '__main__':

    startDrive()

# def start():
  
#     cap = cv2.VideoCapture('kmu_track.mkv')
#     time.sleep(3)

#     while not rospy.is_shutdown():
#         ret, image = cap.read()
#         lpos, rpos = processImage(image)
#         center = (lpos + rpos) / 2
#         # c = center - lpos
#         # c2 = rpos - center
#         # print('c: ', c, 'c2: ', c2)
#         # steer_angle = getSteerAng(pos)
        
#         # draw_steer(frame, steer_angle)
#         if cv2.waitKey(5) & 0xFF == ord('q'):
#             break

# if __name__ == '__main__':

#     start()




