#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : main_drive.py
# 작 성 자 : 신홍재, 황예원, 노현빈
# 생 성 일 : 2021년 07월 10일
####################################################################

import time
import rospy, rospkg
import numpy as np
import cv2
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from detect_line import processImage, posCalibration
from driving_method import (
    getSteerAng,
    getSteerAngOneLine,
    getSteerAng_test,
    getSteerAngOneLine_test,
)
import obstacle_detect
import rear_parking

import sys
import os
import signal

# Variable Initailization


def signal_handler(sig, frame):
    os.system("killall -9 python rosout")
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


def isParking():
    parking_status = False
    lap_num = 0
    # lap_num = getLapNum()
    # FIXME getLapNum : return lap num

    # parking_status = getParkingStatus()
    # FIXME getParkingStatus : if find parking lot >> return True
    #   initialize after some time (using function time.time())
    if lap_num == 3 and parking_status:
        return True
    else:
        return False


def setDrivingMode(is_obstacle):
    if not is_obstacle and not isParking():
        return 0
    elif is_obstacle and not isParking():
        return 1
    elif isParking() is True:
        return 2


def startDrive():
    global pub
    global image
    global cap

    rospy.init_node("main_control")
    pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.sleep(2)
    driving_mode = 0  # (0 :  StandardDriving, 1 : Obstacle, 2 : Parking, 3 : Crosswalk)
    fourcc = cv2.VideoWriter_fourcc(*"DIVX")
    out = cv2.VideoWriter(
        "/home/nvidia/test/" + (str)(time.time()) + ".avi", fourcc, 30.0, (640, 480)
    )
    parking = rear_parking.parking()
    obstacle = obstacle_detect.obstacle()
    test_time = time.time()
    while True:
        while not image.size == (640 * 480 * 3):
            continue
        

        # lpos, rpos, cpos = processImage(image, lane_num_test)
        lpos, rpos, cpos = processImage(image)
        if parking.getUltradata():
            obstacle.setUltrasonic(parking.getUltradata())
        obstacle.obstacleDetect_test()
        out.write(image)
        is_obstacle = obstacle.isObstacle()
        driving_mode = setDrivingMode(is_obstacle)
        speed = 5
        if driving_mode == 0:
            angle = getSteerAng((lpos, rpos), 0)
        elif driving_mode == 1:
            lane_num_test = obstacle.getLaneNum()
            if obstacle.getObstacleNum() == 1:
                lane_num_test += 0.5            
            lpos, cpos, rpos = posCalibration(lane_num_test)
            angle, lpos, rpos = getSteerAngOneLine(
                (lpos, rpos, cpos),
                obstacle.getLaneNum(),
                obstacle.getObstacleLocation(),
            )
            # setPosition(lpos, rpos)
            angle = angle * 2.2
            speed = 6
            # angle = angle * 1.5

        elif driving_mode == 2:
            angle, speed = parking.checkParkingLot()
            angle, speed = 0
        # setAnglenSpeed(angle, speed)
        # stop_flag = obstacle.getStopDriving()
        # if stop_flag is True:
        #     speed = 0
        speed = 5
        setAnglenSpeed(angle,speed)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    out.release()
    rospy.spin()


if __name__ == "__main__":

    startDrive()
