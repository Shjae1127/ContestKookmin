#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : main_drive.py
# 작 성 자 : 신홍재, 황예원
# 생 성 일 : 2021년 07월 10일
####################################################################



import math


# function for calculating steer angle when following center line using pure pursuit
def getSteerAng_center(pos):

    l = 350                     #350mm
    r = 1250                    #1250mm
    ratio = (float)(845/380)
    lpos = pos[0]
    rpos = pos[1]
    print(lpos, rpos)
    theta1_rad = (float)(320-lpos)/(float)(r) * ratio
    theta2_rad = (float)(rpos-320)/(float)(r) * ratio
    ld = (float)(r * math.cos((theta1_rad + theta2_rad)/2))
    alpha_rad = (theta2_rad-theta1_rad)/2
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha_rad))/(float)(ld)))
    degree = -2.5 * degree

    return degree

# function for calculating steer angle when following center, left, right line using pure pursuit
# parameter : 
#   pos : position from line detect [lpos, rpos]
#   lane_num : line want to trace ,   0 : center line , 1 : left line, 2 : right line
# return :
#   angle : unit X, (-50 ~ 50). steer angle for rc car
def getSteerAng(pos, lane_num):

    l = 350                     #350mm
    r = 1250                    #1250mm
    ratio = (float)(845/380)
    lpos = pos[0]
    rpos = pos[1]

    theta1_rad = (float)(320-lpos)/(float)(r) * ratio
    theta2_rad = (float)(rpos-320)/(float)(r) * ratio
    ld_0 = (float)(r * math.cos((theta1_rad + theta2_rad)/2))
    if lane_num == 0:
        ld = ld_0
        alpha_rad = (theta2_rad-theta1_rad)/2
    elif lane_num == 1:
        ld = math.sqrt((pow(1250, 2) * 2 + 2 * pow(ld_0, 2) - pow(((float)(1250) * math.sin((theta1_rad + theta2_rad)/2)), 2))/4)
        alpha_rad = -math.acos(ld_0/(float)(ld))+(theta2_rad-theta1_rad)/2
    
    elif lane_num == 2:
        ld = math.sqrt((pow(1250, 2) * 2 + 2 * pow(ld_0, 2) - pow(((float)(1250) * math.sin((theta1_rad + theta2_rad)/2)), 2))/4)
        alpha_rad = (theta2_rad-theta1_rad)/2 + math.acos(ld_0/(float)(ld))
        
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha_rad))/(float)(ld)))
    angle = -2.5 * degree

    return angle

# function for calculating steer angle when following center, left, right line using pure pursuit
# difference from existing function : camera's viewpoint is equal but translation is used to react more quickly to the curve
def getSteerAng_test(pos, lane_num):

    l = 350                     #350mm
    r = 1250                    #1250mm
    ratio = (float)(845/380)
    lpos = pos[0]
    rpos = pos[1]

    theta1_rad = (float)(320-lpos)/(float)(r) * ratio
    theta2_rad = (float)(rpos-320)/(float)(r) * ratio
    ld_0 = (float)(r * math.cos((theta1_rad + theta2_rad)/2))
    if lane_num == 0:
        ld = ld_0
        alpha_rad = (theta2_rad-theta1_rad)/2
    elif lane_num == 1:
        alpha_rad = math.atan((0.5 * math.tan((theta1_rad + theta2_rad)/2))) - (theta2_rad-theta1_rad)/2
        ld = 0.5 * ld_0/math.cos(alpha_rad + (theta2_rad-theta1_rad)/2)

    elif lane_num == 2:
        ld = math.sqrt((pow(1250, 2) * 2 + 2 * pow(ld_0, 2) - pow(((float)(1250) * math.sin((theta1_rad + theta2_rad)/2)), 2))/4)
        alpha_rad = (theta2_rad-theta1_rad)/2 + math.acos(ld_0/(float)(ld))
    
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha_rad))/(float)(ld)))
    degree = 2.5 * degree
    

    return degree

# function for calculating steer angle when following center, left, right line with one line and center line using pure pursuit
# parameter : 
#   pos : [lpos, rpos, cpos]
#   selected_line : one line you want to trace
#   ignored_line : one line you want to ignore
#   ex) (pos, 1, 1) : 왼쪽 차선을 무시하고 오른측 차선과 중앙 차선을 이용하여 왼쪽 차선을 따라 주행
def getSteerAngOneLine(pos, selected_line, ignored_line):

    l = 350                     #350mm
    r = 1250                    #1250mm
    ratio = (float)(845/380)
    lpos = pos[0]
    rpos = pos[1]
    cpos = pos[2]
    if ignored_line == 2 and selected_line == 1:
        theta1_rad = (float)(cpos-320)/(float)(r) * ratio
        theta2_rad = (float)(320-lpos)/(float)(r) * ratio
        alpha_rad = math.atan(0.5 * math.tan(theta1_rad + theta2_rad)) - theta1_rad
        ld = r * math.cos(theta1_rad + theta2_rad)/math.cos(alpha_rad + theta1_rad)
        # rpos = cpos + (cpos - lpos)
    elif ignored_line == 1 and selected_line == 2:
        theta1_rad = (float)(cpos - 320)/(float)(r) * ratio
        theta2_rad = (float)(rpos - cpos)/(float)(r) * ratio
        alpha_rad = math.atan(0.5 * math.tan(theta2_rad)) + theta1_rad
        alpha_rad = -alpha_rad
        ld = r * math.cos(theta2_rad)/math.cos(alpha_rad - theta1_rad)
        # lpos = cpos - (rpos - cpos)
        
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha_rad))/(float)(ld)))
    degree = 2.5 * degree

    return degree, lpos, rpos


def getSteerAngOneLine_test(pos, selected_line, ignored_line):

    l = 350                     #350mm
    r = 1250                    #1250mm
    ratio = (float)(845/380)
    lpos = pos[0]
    rpos = pos[1]
    cpos = pos[2] 
    if ignored_line == 2 and selected_line == 1:
        theta1_rad = (float)(cpos-320)/(float)(r) * ratio
        theta2_rad = (float)(320-lpos)/(float)(r) * ratio
        alpha_rad = theta2_rad
        
        ld = 0.5 * r
        # rpos = cpos + (cpos - lpos)
    elif ignored_line == 1 and selected_line == 2:
        theta1_rad = (float)(cpos - 320)/(float)(r) * ratio
        theta2_rad = (float)(rpos - cpos)/(float)(r) * ratio
        alpha_0 = math.atan(0.5 * math.tan(theta2_rad)) + theta1_rad
        alpha_rad = 2 * theta1_rad + theta2_rad
        alpha_rad = -alpha_rad
        ld = 0.5 * r
        # lpos = cpos - (rpos - cpos)
        
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha_rad))/(float)(ld)))
    degree = 2.5 * degree

    
    return degree