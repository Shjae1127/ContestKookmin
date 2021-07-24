#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : main_drive.py
# 작 성 자 : 신홍재, 황예원
# 생 성 일 : 2021년 07월 10일
####################################################################



import math



# def getSteerAng(pos):

#     l = 350                     #350mm
#     r = 1250                    #1250mm
#     ratio = (float)(845/380)
#     lpos = pos[0]
#     rpos = pos[1]
#     print(lpos, rpos)
#     theta1_rad = (float)(320-lpos)/(float)(r) * ratio
#     theta2_rad = (float)(rpos-320)/(float)(r) * ratio
#     ld = (float)(r * math.cos((theta1_rad + theta2_rad)/2))
#     alpha_rad = (theta2_rad-theta1_rad)/2
#     degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha_rad))/(float)(ld)))
#     degree = -2.5 * degree

#     return degree

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
    degree = -2.5 * degree
    print(lpos, rpos)

    return degree

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
    print(lpos, rpos)

    return degree

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
        rpos = cpos + (cpos - lpos)
    elif ignored_line == 1 and selected_line == 2:
        theta1_rad = (float)(cpos - 320)/(float)(r) * ratio
        theta2_rad = (float)(rpos - cpos)/(float)(r) * ratio
        alpha_rad = math.atan(0.5 * math.tan(theta2_rad)) + theta1_rad
        alpha_rad = -alpha_rad
        ld = r * math.cos(theta2_rad)/math.cos(alpha_rad - theta1_rad)
        lpos = cpos - (rpos - cpos)
        
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha_rad))/(float)(ld)))
    degree = 2.5 * degree

    return lpos, rpos, degree