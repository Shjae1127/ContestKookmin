#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : main_drive.py
# 작 성 자 : 이성준, 김현진
# 생 성 일 : 2021년
####################################################################

import time, rospy
from common_ros_cls import ultra_module


class parking:
    def __init__(self):
        self.rback_prev = 0
        self.right_prev = 0
        self.mode = 0
        self.stage = 0
        self.init0, self.init1, self.init2 = False, False, False
        self.parking_start_time = 0
        self.t_point1, self.t_point2 = 0, 0
        self.is_parking_launch = False
        self.parking = ultra_module("rear_parking")
        self.left, self.right, self.rback, self.back, self.lback = 0, 0, 0, 0, 0

    def sortUltraData(self):
        ultra_data = self.parking.get_ultrasonic_data()
        ultra_val = ultra_data[0:8]
        self.left = ultra_val[0]
        self.right = ultra_val[4]
        self.rback = ultra_val[5]
        self.back = ultra_val[6]
        self.lback = ultra_val[7]

    def getUltradata(self):
        
        # ultra_val = ultra_data[0:8]
        # return [ultra_data[0], ultra_data[4], ultra_data[5], ultra_data[7]]
        return self.parking.get_ultrasonic_data()

    def launchParkingMode(self):
        self.sortUltraData()
        if self.is_parking_launch is False:
            self.parking_start_time = time.time()
            angle, speed = 0, 0
            self.is_parking_launch = True
            print("test")
            return angle, speed
        # method 1. control by time
        parking_time = time.time() - self.parking_start_time

        if parking_time < 1.3:
            angle, speed = 0, 5

        elif parking_time < 2.5:  # 1sec
            angle, speed = -40, 0

        elif parking_time < 4.8:  # 1.6sec
            # if back < 80:
            #     return -20, 0
            angle, speed = -40, -4

        elif parking_time < 5.8:  # 1sec
            angle, speed = 50, 0

        elif parking_time < 8.1:  # 1.6sec
            # if back < 80:
            #     return -20, 0
            angle, speed = 50, -4

        elif parking_time < 10.1:
            angle, speed = 0, 0

        return angle, speed

    # mehtod 2. control bt ultra val
    # if stage == 0:
    #     angle = -40
    #     speed = -4
    #         if 60 < rback < 80:
    #             angle = 0
    #             speed = -4
    #             stage = 1
    #     return angle, speed

    # if stage == 1:

    # elif back > 15:
    #     return 0, -3

    # return angle, speed

    # if parking_time < 1.0:
    #     return 0, 5

    # elif parking_time < 2.0: # 1sec
    #     return -40, 0

    # elif parking_time < 3.0: # 1s.5ec
    #     return -40, -4

    # elif parking_time < 4.0: # 1sec
    #     return 0, 0

    # elif parking_time < 4.5: # 1sec
    #     return 0, -4

    # elif parking_time < 5.5: # 1sec
    #     return 50, 0

    # elif parking_time < 6.5: # 1sec
    #     return 50, 0

    # return angle, speed

    # angle : R => 50 : 132 -40 : 132 -45 : 112

    def checkParkingLot(self):
        self.sortUltraData()
        if self.init0 is False:  # initialize
            self.right_prev = self.right
            self.rback_prev = self.rback
            self.init0 = True

        if self.mode == 0:
            print(
                "mode 0 : line tracing",
                " right : ",
                self.right,
                "rback_prev : ",
                self.rback_prev,
                "rback : ",
                self.rback,
            )
            if self.right_prev - self.right >= 20:
                if self.init1 == False:
                    self.t_point1 = time.time()
                    self.init1 = True
                # print('time checking start')

            if abs(self.rback_prev - self.rback) >= 20:  # check for mode 1
                self.mode = 1
                # print("mode : ", mode)

        if self.mode == 1:  # check for mode 2
            print(
                "mode 1 : line tracing, ",
                "right_prev : ",
                self.right_prev,
                " right : ",
                self.right,
            )

            if self.right_prev - self.right >= 10:
                if self.init2 == False:
                    self.t_point2 = time.time()
                    self.init2 = True
                self.mode = 2
                print("duration of time : ", self.t_point2 - self.t_point1)

        if self.mode == 2:  # start parking mode

            angle, speed = self.launchParkingMode()
            # print('mode 2 : parking pursuit , ', 'right_prev : ' , right_prev, ' r : ' ,right , 'parking time : ', time.time()- park_start_time)
            # print("time duration : ", t_point2 - t_point1)
            print(
                "rback val : ",
                self.rback,
                "back val : ",
                self.back,
                "left val : ",
                self.lback,
            )
            return angle, speed  # parking code

        self.rback_prev = self.rback
        self.right_prev = self.right

        if self.mode < 2:
            return 0, 5  # (angle, speed) # pure pursuit code


# def action_time_init(self):
#     self.action_start_time = time.time()