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
        self.version = 0
        self.stage = 0
        self.init0, self.init1, self.init2 = False, False, False
        self.parking_start_time = 0
        self.parking_time = time.time()
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
        global parking_time

        self.sortUltraData()
        if self.is_parking_launch is False:
            self.parking_start_time = time.time()
            #angle, speed = 0, 0 #
            self.is_parking_launch = True
            #print("test")
            #return angle, speed
        # 1. control by time - into the parking lot
        parking_time = time.time() - self.parking_start_time

        # if parking_time < 2.3: #par
        #     angle, speed = 0, 3

        if parking_time < 3.5:  
            angle, speed = -40, 0

        elif parking_time < 5.8: #par
            angle, speed = -40, -3

        elif parking_time < 6.2:
            angle, speed = 0, -3

        # 2. rback sensor - linear

        elif parking_time < 8:  
            if self.rback > 45: #par
               angle, speed = 0, -3
            else:
                angle, speed = 40, 0 

        # 3. back sensor - left curve
         
        elif parking_time < 12: 
            if self.back > 15: #par
                angle, speed = 40, -3
            else:
                angle, speed = 0, 0    

        # 4. right sensor - x coord , orientation callib

        elif parking_time < 17:
            if self.right > 15: #par
                if self.back < 40: #par
                    angle, speed = -20, 3 #3
                elif self.back > 40:
                    angle, speed = 0, 0

            else :
                if self.lback - self.rback > 3 and self.back < 57: #par
                    angle, speed = 10, 3 #3
                elif self.right < 13 and self.back < 57: #par
                    angle, speed = 10, 3
                else :
                    angle, speed = 0, 0

        # 5. back sensor - y coord

        elif parking_time <20:
            if self.back < 46: # par
                angle, speed = 0, 3 #3
            else:
                angle, speed = 0, 0 

        elif parking_time < 23:
            if self.back > 46: # par
                angle, speed = 0, -3 #-3
            else:
                angle, speed = 0, 0

        else:
            angle, speed = 0, 0             

        print("rback : {} back : {}, lback : {}, right : {}". format(self.rback, self.back, self.lback, self.right))
        print("time : ", parking_time)

        return angle, speed, parking_time

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

            self.version = 10

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

            self.version = 20

            if self.right_prev - self.right >= 15:
                if self.init2 == False:
                    self.t_point2 = time.time()
                    self.init2 = True
                self.mode = 2

                print("duration of time : ", self.t_point2 - self.t_point1)
            
            

        if self.mode == 2:  # start parking mode
            self.version = 30
            # angle, speed = self.launchParkingMode()
            # print('mode 2 : parking pursuit , ', 'right_prev : ' , self.right_prev, ' r : ' ,self.right)
            # print("time duration : ", t_point2 - t_point1)
            # print(
            #     "rback val : ",
            #     self.rback,
            #     "back val : ",
            #     self.back,
            #     "left val : ",
            #     self.left,
            # )

        self.rback_prev = self.rback
        self.right_prev = self.right

        return self.version

        # if self.mode < 2:
        #     return angle, speed
        # return 0, 3  # (angle, speed) # pure pursuit code


# def action_time_init(self):
#     self.action_start_time = time.time()