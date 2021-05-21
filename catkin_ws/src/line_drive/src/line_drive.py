#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time

Width = 640
Height = 480
Offset = 320
isNotLpos = True
isNotRpos = True
noPos = True
pathwidth = 400
standardDriving = True
# draw rectangle

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)   
    return img

# You are to find "left and light position" of road lanes


def detect_line(frame):
    global standardDriving

    x, w, h = 30, 600, 50
    threshold, minLineLength, maxLineGap = 50, 5, 100
    num_L, num_R = 0, 0
    line_L_X1, line_L_Y1 = 0, 0
    line_L_X2, line_L_Y2 = 0, 0
    line_R_X1, line_R_Y1 = 0, 0
    line_R_X2, line_R_Y2 = 0, 0
    frame = cv2.Canny(frame, 400, 200, None, 3)
    if standardDriving:
        w = 240
        x_L = x
        x_M = x+w
        x_R = x+w+100
        roi_L = frame[Offset:Offset+h, x_L:x_L+w]
        roi_M = frame[Offset:Offset+h, x_M:x_M+100]
        roi_R = frame[Offset:Offset+h, x_R:x_R+w]
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        linesP_L = cv2.HoughLinesP(
            roi_L, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)
        linesP_M = cv2.HoughLinesP(
            roi_M, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)
        linesP_R = cv2.HoughLinesP(
            roi_R, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)
        if linesP_L is not None:
            for i in range(0, len(linesP_L)):
                l = linesP_L[i][0]
                line_L_X1 = line_L_X1 + x_L+l[0]
                line_L_Y1 = line_L_Y1 + Offset+l[1]
                line_L_X2 = line_L_X2+x_L+l[2]
                line_L_Y2 = line_L_Y2+Offset+l[3]
                num_L = num_L + 1

        if linesP_M is not None:
            for i in range(0, len(linesP_M)):
                l = linesP_M[i][0]

        if linesP_R is not None:
            for i in range(0, len(linesP_R)):
                l = linesP_R[i][0]
                line_R_X1 = line_R_X1 + x_R+l[0]
                line_R_Y1 = line_R_Y1 + Offset+l[1]
                line_R_X2 = line_R_X2+x_R+l[2]
                line_R_Y2 = line_R_Y2+Offset+l[3]
                num_R = num_R + 1
        if num_L != 0:
            line_L_X1, line_L_Y1 = line_L_X1/num_L,  line_L_Y1/num_L
            line_L_X2, line_L_Y2 = line_L_X2/num_L, line_L_Y2/num_L
            cv2.line(frame, (line_L_X1, line_L_Y1), (line_L_X2,
                                                     line_L_Y2), (0, 0, 255), 3, cv2.LINE_AA)
        if num_R != 0:
            line_R_X1, line_R_Y1 = line_R_X1/num_R,  line_R_Y1/num_R
            line_R_X2, line_R_Y2 = line_R_X2/num_R, line_R_Y2/num_R
            cv2.line(frame, (line_R_X1, line_R_Y1), (line_R_X2,
                                                     line_R_Y2), (0, 0, 255), 3, cv2.LINE_AA)

        return frame


def process_image(frame):
    global Offset
    global noPos
    global isNotLpos
    global isNotRpos
    global pathwidth
    point = 0
    frame = detect_line(frame)
    lpos, rpos = 0, 0
    # frame = cv2.Canny(frame, 400, 200, None, 3)

    # x = 0
    # w = 600
    # h = 50
    # roi = frame[Offset:Offset+h, x:x+w]
    # if noPos:
    #     lpos, rpos = 100, 500
    #     noPos = False
    # frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    # threshold = 50
    # minLineLength = 5
    # maxLineGap = 100
    # linesP = cv2.HoughLinesP(
    #     roi, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)
    # if linesP is not None:
    #     for i in range(0, len(linesP)):
    #         l = linesP[i][0]
    #         cv2.line(frame, (x+l[0], Offset+l[1]), (x+l[2],
    #                                                 Offset+l[3]), (0, 0, 255), 3, cv2.LINE_AA)
    #         if (l[0] != l[2]):
    #             slope = (float)(l[1]-l[3])/(float)(l[0]-l[2])
    #             point = x + l[0]+(h/2-l[1])/slope

    #             if point < 300:
    #                 lpos = int(round(point))
    #                 isNotLpos = False
    #             elif point > 400:
    #                 rpos = int(round(point))
    #                 isNotRpos = False
    #     if isNotLpos:
    #         lpos = rpos - pathwidth
    #         isNotLpos = False
    #     if isNotRpos:
    #         rpos = lpos + pathwidth
    #         isNotRpos = False
    #     if (not isNotLpos) & (not isNotRpos):
    #         pathwidth = rpos - lpos
    #     frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    #     isNotLpos = True
    #     isNotRpos = True

    return (lpos, rpos), frame


def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

    arrow_pic = cv2.imread('steer_arrow.png', cv2.IMREAD_COLOR)

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 1.5, 0.7)    
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2)
           : (Width/2 + arrow_Width/2)] = res

    cv2.imshow('steer', image)

# You are to publish "steer_anlge" following load lanes
if __name__ == '__main__':
    cap = cv2.VideoCapture('kmu_track.mkv')
    time.sleep(3)

    while not rospy.is_shutdown():
        ret, image = cap.read()
        pos, frame = process_image(image)
        
        steer_angle = 0
        draw_steer(frame, steer_angle)

        if cv2.waitKey(3) & 0xFF == ord('q'):
            break

