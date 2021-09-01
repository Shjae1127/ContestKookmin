#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy, rospkg
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

import sys
import os
import signal


class crosswalkQueue:
    def __init__(self):
        self.data = [0]

    def push(self, item):
        self.data.append(item)

    def pop(self):
        return self.data.pop(0)

class flagQueue:
    def __init__(self):
        self.data = [0]*2

    def push(self, item):
        self.data.append(item)

    def pop(self):
        return self.data.pop(0)


class Crosswalk:
    def __init__(self):
        self.stopline = 125
        self.area = 2000
        self.length = 300
        self.que = crosswalkQueue()
        self.flag_que = flagQueue()
        self.flag = 0
        self.redflag = False
        self.is_detected = False

    def changeView(self, img):
    # Calibration
        img_size = (640, 480)
        warp_img_w, warp_img_h, warp_img_mid = 650, 120, 60

        mtx = np.array([[363.090103, 0.000000, 313.080058],
                                [0.000000, 364.868860, 252.739984],
                                [0.000000, 0.000000, 1.000000]])
        dist = np.array([-0.334146, 0.099765, -0.000050, 0.001451, 0.000000])
        cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, img_size, 1, img_size)

        # Bird Eye View
        warpx_mid, warpx_margin_hi, warpx_margin_lo, warpy_hi, warpy_lo, tilt = 320, 200, 319, 325, 375, -5
        warp_src  = np.array([[warpx_mid+tilt-warpx_margin_hi, warpy_hi], [warpx_mid+tilt+warpx_margin_hi, warpy_hi], 
                                    [warpx_mid-warpx_margin_lo,  warpy_lo], [warpx_mid+warpx_margin_lo, warpy_lo]], dtype=np.float32)
        warp_dist = np.array([[100, 0], [649-100, 0],
                                    [100, 119], [649-100, 119]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(warp_src, warp_dist)
        
        img = cv2.undistort(img, mtx, dist, None, cal_mtx)
        # cv2.imshow('calibrated', img)

        img = cv2.warpPerspective(img, M, (warp_img_w, warp_img_h), flags=cv2.INTER_LINEAR)

        return img

    def isCrossWalk(self, image):

        # cv2.imshow('origin', image)
        view = self.changeView(image)
        blur = cv2.GaussianBlur(view, (5, 5), 0)
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        _, lane = cv2.threshold(L, self.stopline, 255, cv2.THRESH_BINARY)
        _, contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # drive(0, 20)
        self.is_detected = False
        for cont in contours:
            l = cv2.arcLength(cont, True)
            a = cv2.contourArea(cont)

            # 쓸데없는거 cut
            if not ((a > self.area) and (l > self.length)):
                continue
            if len(cv2.approxPolyDP(cont, self.length*0.02, True)) != 4:
                continue

            x, y, w, h = cv2.boundingRect(cont)

            # 정지선 아닌 다른 노이즈 검출 방지용
            if w > h*3 :
                cv2.rectangle(view, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.is_detected = True

        # if self.is_detected is True:
        #         self.que.push(1)
        # elif self.is_detected is False:
        #         self.que.push(0)
        # if len(self.que.data) >= 2 :
        #         self.que.pop()
        
        # for flag in self.que.data :
        #     if flag == 1:
        #         self.flag = 1
        #         self.flag_que.push(1)
        #         # print("Crosswalk")
        #         # print(self.flag_que.data)
        #         break
        # if self.flag == 0:
        #     self.flag_que.push(0)

        # if self.flag_que.pop() - self.flag == 1:
        #     self.redflag = True
        #     # print("----------------------------------------redFlag")
        # else:
        #     self.redflag = False
        # self.flag = 0
        cv2.imshow('stopline', view)
        return self.is_detected


        # 적절한 감속 법 필요
        # 더이상 detect 못하는 지점(정지선으로부터 대략 30cm 앞)
        # if detected > 100 & flag == False :
            
        #     count += 1

        #     if count > 10 & count <= 20:
        #         drive(0, 10)
        #     elif count > 20 & count <= 30:
        #         drive(0, 5)
        #     elif count > 30:
        #         drive(0, 0)
        #     else:
        #         drive(0, 15)
