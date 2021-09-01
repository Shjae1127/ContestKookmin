#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : main_drive.py
# 작 성 자 : 신홍재, 황예원
# 수 정 자 : 노현빈
# 생 성 일 : 2021년 07월 10일
# 수 정 일 : 2021년 09월 01일
####################################################################
import cv2
import numpy as np
import math
import os
import pickle


lpos, rpos, cpos = 125, 500, 320
lpos_prev, rpos_prev = lpos, rpos
cpos_prev = cpos
pathwidth = rpos - lpos
Gap = 50
Width = 640
Height = 480
Offset = 320
start_obstacle = 0

# 중앙 차선 좌표도 그릴 수 있도록 수정
# draw rectangle
# def drawRectangle(img, lpos, rpos, cpos, offset=0):
def drawRectangle(img, offset=0):
    # center = (lpos + rpos) / 2
    global lpos, rpos, cpos
    center = cpos

    cv2.rectangle(img, (lpos - 5, 15 + offset), (lpos + 5, 25 + offset), (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset), (rpos + 5, 25 + offset), (0, 255, 0), 2)
    cv2.rectangle(
        img, (center - 5, 15 + offset), (center + 5, 25 + offset), (0, 255, 0), 2
    )
    # cv2.rectangle(img, (lpos - 50, -50 + offset), (lpos + 50, 50 + offset), (0, 255, 0), 2)
    # cv2.rectangle(img, (rpos - 50, -50 + offset), (rpos + 50, 50 + offset), (0, 255, 0), 2)
    # cv2.rectangle(
    #     img, (center - 5, 15 + offset), (center + 5, 25 + offset), (0, 255, 0), 2
    # )
    cv2.rectangle(img, (320 - 5, offset + 15), (320 + 5, offset + 25), (0, 0, 255), 2)
    print(lpos, center, rpos)
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


# Get the positions of left lane, right lane
def detectLine(frame):
    global lpos, rpos, cpos, pathwidth, lpos_prev, rpos_prev, cpos_prev, start_obstacle
    #####  Variable Initailization  #####
    src = frame
    w, h = 150, 50
    # threshold, minLineLength, maxLineGap = 50,5,100
    # threshold, minLineLength, maxLineGap = 25,5,100
    threshold, minLineLength, maxLineGap = 10,1,100
    # threshold, minLineLength, maxLineGap = 50,1,100
    # num_L, num_R = 0, 0
    num_L, num_R = 1, 1
    line_L_X1, line_L_Y1 = 0, 0
    line_L_X2, line_L_Y2 = 0, 0
    line_R_X1, line_R_Y1 = 0, 0
    line_R_X2, line_R_Y2 = 0, 0

    ##### Hough Line Transform #####
    mark = np.copy(frame)
    mark_Gray = cv2.cvtColor(mark,cv2.COLOR_BGR2GRAY)
    ###############################\
    blk_size = 7
    C=10
    # t,t_otsu = cv2.threshold(mark_Gray,-1,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
    # print('otsu threshold',t)
    # k = np.array([[1,1],[1,1]])*(1/4)
    # t_otsu = cv2.filter2D(t_otsu,-1,k)
    # cv2.imshow('otsu',t_otsu)
    mark_thresh = cv2.adaptiveThreshold(mark_Gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,blk_size,C)
    mark = cv2.bitwise_not(mark_thresh)
    cv2.imshow('test', mark)


    # mark = cv2.bitwise_and(mark, mark, mask = thresh_img)
    ################################
    frame = cv2.Canny(
        frame, 400, 200, None, 3
    )  # detect the edges of image by using Canny detector
    # cv2.imshow('raw',frame)
    w = 100
    x_L = lpos - w / 2
    x_R = rpos - w / 2
    if x_L < 0:
        x_L = 0
    if x_R > 640:
        x_R = 640
    if x_R < x_L:
        x_R = x_L + 300
    x_R = rpos - w / 2
    x_C = cpos - w

    ### Set ROI tracing lpos, and rpos  ###
    roi_L = mark[Offset : Offset + h, x_L : x_L + w]
    roi_R = mark[Offset : Offset + h, x_R : x_R + w]
    # roi_C = mark[340 : 340 + Gap, 100 : Width - 100]  # 센터라인용 roi인데 실제로 돌려보고 조정해도 괜츈(roi더 넓게 하되 이전 좌표와의 편차로 벗어나는 값들 거르기)
    roi_C = mark[340:340+Gap, x_C : x_C + w * 2]


    linesP_L = cv2.HoughLinesP(
        roi_L, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap
    )  # run Probablistic Hough Line Transform for detect the line of left side
    linesP_R = cv2.HoughLinesP(
        roi_R, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap
    )  # run Probablistic Hough Line Transform for detect the line of right side
    # For stability, calculate the average position
    # if linesP_L is not None:
    #     for i in range(0, len(linesP_L)):
    #         l = linesP_L[i][0]
    #         line_L_X1 = line_L_X1 + x_L + l[0]
    #         line_L_Y1 = line_L_Y1 + Offset + l[1]
    #         line_L_X2 = line_L_X2 + x_L + l[2]
    #         line_L_Y2 = line_L_Y2 + Offset + l[3]
    #         num_L = num_L + 1

    # if linesP_R is not None:
    #     for i in range(0, len(linesP_R)):
    #         l = linesP_R[i][0]
    #         line_R_X1 = line_R_X1 + x_R + l[0]
    #         line_R_Y1 = line_R_Y1 + Offset + l[1]
    #         line_R_X2 = line_R_X2 + x_R + l[2]
    #         line_R_Y2 = line_R_Y2 + Offset + l[3]
    #         num_R = num_R + 1

    # # FIXME if lpos or rpos change extremly ignore current lpos, rpos and load previous value
    # # For accuracy, update lpos and rpos only if 0 < number of detected lines < 3
    # if (num_L > 0) & (num_L < 3):
    #     line_L_X1, line_L_Y1 = line_L_X1 / num_L, line_L_Y1 / num_L
    #     line_L_X2, line_L_Y2 = line_L_X2 / num_L, line_L_Y2 / num_L
    #     lpos = (Offset + h / 2 - line_L_Y1) * (line_L_X1 - line_L_X2) / (
    #         line_L_Y1 - line_L_Y2
    #     ) + line_L_X1
    #     if abs(lpos - lpos_prev) > 150:
    #         lpos = lpos_prev

    # if (num_R > 0) & (num_R < 3):
    #     line_R_X1, line_R_Y1 = line_R_X1 / num_R, line_R_Y1 / num_R
    #     line_R_X2, line_R_Y2 = line_R_X2 / num_R, line_R_Y2 / num_R
    #     rpos = (Offset + h / 2 - line_R_Y1) * (line_R_X1 - line_R_X2) / (
    #         line_R_Y1 - line_R_Y2
    #     ) + line_R_X1
    #     if abs(rpos - rpos_prev) > 150:
    #         rpos = rpos_prev

    #########################################################################################FIXME#####################################
    if linesP_L is not None:
        for i in range(0, len(linesP_L)):
            l = linesP_L[i][0]
            line_L_X1 = x_L + l[0]
            line_L_Y1 = Offset + l[1]
            line_L_X2 = x_L + l[2]
            line_L_Y2 = Offset + l[3]

            lpos_temp = (Offset + h / 2 - line_L_Y1) * (line_L_X1 - line_L_X2) / (line_L_Y1 - line_L_Y2) + line_L_X1
            if x_L <= lpos_temp <= x_L + w:
                lpos = lpos_temp + lpos
                num_L = num_L + 1

    if linesP_R is not None:
        for i in range(0, len(linesP_R)):
            l = linesP_R[i][0]
            line_R_X1 = x_R + l[0]
            line_R_Y1 = Offset + l[1]
            line_R_X2 = x_R + l[2]
            line_R_Y2 = Offset + l[3]

            rpos_temp = (Offset + h / 2 - line_R_Y1) * (line_R_X1 - line_R_X2) / (line_R_Y1 - line_R_Y2) + line_R_X1
            if x_R <= rpos_temp <= x_R + w:
                rpos = rpos_temp + rpos
                num_R = num_R + 1
    if num_L == 0 or num_L >= 10:
        lpos = lpos_prev
    else:
        lpos = lpos/num_L
    if num_R == 0 or num_R >= 10:
        rpos = rpos_prev

    else:
        rpos = rpos/num_R
    
    print(num_L, num_R)
    


    #########################################################################################

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
    x_r, y_r, w_r, h_r = getRectangle(roi_C)  # 여기서 받은건 현재 잘라놓은 roi_C에서의 좌표

    # To avoid 엣지 이미지 노이즈때문에 차도에서 관계없는 점 검출됨 and 좌우 차선 부근에서 검출됨
    if w_r > 10 and h_r > 10 and (h_r + 4) >= w_r:  # 여기 사각형 크기나 높이, 너비 비율은 유동적으로 조절해야할듯
        x_r += x_C
        y_r += Offset
        # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)

        cpos = x_r + w_r / 2  # 센터 바운딩박스 중간점
        if not lpos + 50 < cpos < rpos - 50:
            cpos = (lpos + rpos) / 2
        # frame = draw_rectangle(frame, lpos, rpos, cpos, offset=Offset)

    # else:
    #     frame = draw_rectangle(frame, lpos, rpos, cpos, offset=Offset)
    lpos_prev = lpos
    rpos_prev = rpos

    cpos_prev = cpos
    return src, lpos, rpos, cpos


# calibration function
def undistortImage(
    frame, cal_dir="/home/nvidia/xycar_ws/src/mainstage/src/data4_python2.pickle"
):

    with open(cal_dir, mode="rb") as f:
        file = pickle.load(f)

    mtx = file["mtx"]
    dist = file["dist"]

    undistored_img = cv2.undistort(frame, mtx, dist, None, mtx)

    return undistored_img


def processImage(frame):
    global lpos, rpos, cpos
    # frame = distorted frame, undistorted_frame = calibrated frame
    undistorted_frame = undistortImage(frame)
    # undistorted_frame, lpos, rpos, cpos = detectLine(undistorted_frame)  # 센터 좌표 추가
    frame, lpos, rpos, cpos = detectLine(frame)  # 센터 좌표 추가
    if lpos < 0:
        lpos = 0
    if rpos > 640:
        rpos = 640
    if rpos-cpos <60 :
        cpos = rpos - 160
        print("hahah")
    # undistorted_frame = drawRectangle(undistorted_frame, offset=Offset)  # 센터 좌표 추가
    frame = drawRectangle(frame, offset=Offset)  # 센터 좌표 추가    

    # cv2.imshow('image', frame)
    # cv2.imshow("cal", undistorted_frame)
    cv2.imshow("cal", frame)

    return (lpos, rpos, cpos)


# function for setting lpos, rpos

"""
보정value 건드리지마셈 
바꾸면 장애물 회피 니가 다하셈
+
speed = 6
angle = angle * 2.2
장애물 value 고정임
"""
def posCalibration(lane_num,obstacle_num):
    global lpos, rpos, cpos, start_obstacle
    #obstacle num 받아야할거 같음
    # 첫번째 중앙에서 시작하는데 보정 다르게 들어가야함
    if lane_num == 1.5:#첫번째 장애물 피하기 위한 변수 설정
        start_obstacle = 2
        lane_num = 1
    elif lane_num == 2.5:
        start_obstacle = 1
        lane_num = 2
    print('start_obstacle',start_obstacle,'lanenum{}'.format(lane_num))
    
    #lpos,cpos,rpos 160만큼 차이남 
    #첫번째 장애물 위치에 따라 보정 다 다르게? 케이스 두개로 해서?
    if start_obstacle == 1: #마지막 장애물에서 보정(마지막 장애물 위치는 처음 장애물 위치와 same)
        if lane_num == 0 and (rpos - cpos < 100 or rpos - cpos > 250):
            cpos = lpos + 140 
            rpos = lpos + 280
            # print("---------------------------------yahoo1------------------------------------")
        elif lane_num == 2:
            if obstacle_num !=3 :
                if rpos - cpos <160 : 
                    cpos = rpos - 160
                    lpos = cpos - 160
                    # print("---------------------------------yahoo2------------------------------------")
                elif rpos - cpos >180 :
                    cpos = rpos -170
                    lpos = cpos -170
                    # print("---------------------------------yahoo2------------------------------------")
                
            else :
                if rpos - cpos <170 :
                    cpos = rpos -170
                    lpos = cpos -160
                    # print("---------------------------------yahoo3------------------------------------")
                elif rpos - cpos >180 or cpos - lpos <90 :
                    cpos = rpos - 150
                    lpos = cpos - 160
                    # print("---------------------------------yahoo3------------------------------------")


        elif lane_num == 1 :
            if cpos - lpos < 160 :
                cpos = lpos +170
                rpos = cpos + 160
                # print("---------------------------------yahoo4------------------------------------")
            elif cpos - lpos > 180 :
                cpos = lpos +180
                rpos = cpos+180
                # print("---------------------------------yahoo4------------------------------------")
        
        # elif lane_num == 2 and (cpos - lpos < 80 or cpos - lpos >250 )
    if start_obstacle == 2:
        if lane_num == 0 and (cpos - lpos < 100 or cpos - lpos > 250):
            cpos = lpos + 140 #lpos?
            rpos = lpos + 280
            # print("---------------------------------google1------------------------------------")
        elif lane_num == 1 :
            if obstacle_num !=3 :
                if cpos - lpos < 160 : 
                    cpos = lpos + 160
                    rpos = cpos + 160
                    # print("---------------------------------google2------------------------------------")
                
                elif cpos - lpos > 170 :
                    cpos = lpos +170
                    rpos = cpos +170
                    # print("---------------------------------google2------------------------------------")

            else :
                if cpos- lpos < 160 :
                    cpos = lpos +160
                    rpos = cpos +160
                    # print("---------------------------------google4------------------------------------")                    
                elif cpos - lpos > 170 or rpos- cpos < 90 : #첫번째 장애물이 2차선에 있을 때 보정
                    cpos = lpos + 132
                    rpos = cpos + 170 
                    # print("---------------------------------google5------------------------------------")


            # cpos = lpos + 140
            # rpos = lpos + 280
        elif lane_num == 2 : 
            if rpos - cpos < 160 : #두번째 장애물 1차선에 있을 때 보정
                cpos = rpos - 170
                lpos = cpos - 160
                # print("---------------------------------google4------------------------------------")

            elif rpos - cpos > 180:
                cpos = rpos - 180
                lpos = cpos - 180
                # print("---------------------------------google4------------------------------------")

            # cpos = rpos - 140
            # lpos = rpos - 280
            # elif lane_num == 1 and (cpos - lpos < )


    
    return lpos, cpos, rpos