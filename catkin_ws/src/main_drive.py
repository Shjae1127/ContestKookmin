#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : main_drive.py
# 작 성 자 : 신홍재, 황예원, 노현빈
# 생 성 일 : 2021년 07월 10일
####################################################################

import time

from numpy.core.numeric import cross
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
)
import obstacle_detect
import rear_parking
import ped_stop

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

class Flag:
    def __init__(self):
        self.parking = False
        self.obstacle = False
        self.standard = False
        self.crosswalk = False
        self.lap = 0

# publish xycar_motor msg
def setAnglenSpeed(Angle, Speed):
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)


def isParking(flag):
    parking_status = False
    # lap_num = getLapNum()
    # FIXME getLapNum : return lap num

    # parking_status = getParkingStatus()
    # FIXME getParkingStatus : if find parking lot >> return True
    #   initialize after some time (using function time.time())
    if flag.lap == 3 and parking_status:
        return True
    else:
        return False
    

def setDrivingMode(flag):
    
    if flag.obstacle and not isParking(flag):
        return 1
    elif isParking(flag) is True:
        return 2
    elif flag.crosswalk is True:
        return 3
    else:
        return 0


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
    # crosswalk = ped_stop.Crosswalk()
    flag = Flag()
    crosswalk_time = 0
    while True:
        while not image.size == (640 * 480 * 3):
            continue

        lpos, rpos, cpos = processImage(image)
        if parking.getUltradata():
            obstacle.setUltrasonic(parking.getUltradata())
        obstacle.obstacleDetect()
        out.write(image)
        flag.obstacle = obstacle.isObstacle()
        # if obstacle.getFinishFlag() is True:
        #     print("-----Searching Crosswalk-------")
        #     if crosswalk:
        #         crosswalk = ped_stop.Crosswalk()
        #     if crosswalk.isCrossWalk(image) is True and crosswalk_time == 0:
        #         crosswalk_time = time.time()
        #         flag.lap += 1

        #     if time.time() - crosswalk_time < 6:
        #         flag.crosswalk = True
        #     elif time.time() - crosswalk_time == 6:
        #         flag.crosswalk = False
        #         obstacle.resetFinishFlag()
        driving_mode = setDrivingMode(flag)
        # driving_mode = 2
        print("driving mode : " + str(driving_mode))
        
        if driving_mode == 0:
            angle = getSteerAng((lpos, rpos), 0)
            angle = angle * 2
            speed = 6
            set_time = time.time()
        elif driving_mode == 1:
            test_lane_num = obstacle.getLaneNum()
            if obstacle.getObstacleNum() == 1 :
                test_lane_num +=0.5 
            lpos, cpos, rpos = posCalibration(test_lane_num,obstacle.getObstacleNum())
            angle, lpos, rpos = getSteerAngOneLine(
                (lpos, rpos, cpos),
                obstacle.getLaneNum(),
                obstacle.getObstacleLocation(),
            )
            
            speed = 6
            angle = angle * 2.2
            # angle = angle * 5

        elif driving_mode == 2:

            park_mode = parking.checkParkingLot() 

            if park_mode == 10 or 20:
                angle = getSteerAng((lpos, rpos), 0)
                angle = angle * 2.1

                if park_mode == 10:
                    speed = 5
                else:
                    speed = 3

                print("mode : {}, speed : {}, angle : {}".format(park_mode, speed, angle))

            if park_mode == 30:

                angle, speed, parking_time = parking.launchParkingMode()
                print("time : ", parking_time)

                if parking_time < 2.6:
                    angle = getSteerAng((lpos, rpos), 0)
                    angle = angle * 2.1

                    speed = 3
                    
                else:
                    print("moooode : {}, speed : {}, angle : {}".format(park_mode, speed, angle))

        elif driving_mode == 3:

            speed = 0

        setAnglenSpeed(angle,speed)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    out.release()
    rospy.spin()


if __name__ == "__main__":

    startDrive()
