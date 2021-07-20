#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : obstacle_detect.py
# 수 정 자 : 신홍재,노현빈
# 생 성 일 : 2021년 07월 16일
# 수 정 일 : 2021년 07월 17일
####################################################################

import rospy, time
from common_ros_cls import ultra_module, lidar_module
from driving_method import getSteerAng


class obstacle:
    def __init__(self):
        # Variable Initialization
        self.obstacle_search_status = True
        self.obstacle_num = 0
        self.obstacle_location = 0  # (0 : None, 1 : Left, 2: Right)
        self.lidar = lidar_module("obstacle_detect")
        self.starting_time = 0
        self.lane_num = 0  # (0 : 중앙, 1 : 왼쪽 차선, 2 : 오른쪽 차선)

        
    def obstacleDetect(self):
        stack_L, stack_C, stack_R = 0, 0, 0
        reset_flag = 0
        obstacle_threshold = 10
        if self.obstacle_search_status:
            self.starting_time = time.time()
        lidar_range, lidar_angleInc = self.lidar.getLidarData()
        if self.obstacle_search_status & (self.obstacle_num == 0):
            for i in range(55, 125):
                if lidar_range[i] < 0.9:
                    if 55 <= i <= 90:
                        stack_L = stack_L + 1
                    elif 91 <= i <= 125:
                        stack_R = stack_R + 1
                if stack_L >= obstacle_threshold / 2:
                    stack_R = 0
                if stack_R >= obstacle_threshold / 2:
                    stack_L = 0
                if stack_L >= obstacle_threshold:
                    self.obstacle_location = 1
                    self.obstacle_search_status = False
                    self.obstacle_num = 1
                if stack_R >= obstacle_threshold:
                    self.obstacle_location = 2
                    self.obstacle_search_status = False
                    self.obstacle_num = 1
            reset_flag = reset_flag + 1
            if reset_flag == 3:
                stack_L, stack_R, reset_flag = 0, 0, 0
        elif self.obstacle_search_status & (self.obstacle_num != 0):
            for i in lidar_range[85:95]:
                if i < 0.9:
                    stack_C = stack_C + 1
            reset_flag = reset_flag + 1
            if stack_C > obstacle_threshold:
                self.obstacle_location = self.lane_num
            if reset_flag == 3:
                reset_flag, stack_C = 0, 0
        print(self.obstacle_location)


    def obstacleDetect_test(self):
        stack_L, stack_C, stack_R = 0, 0, 0
        reset_flag = 0
        obstacle_threshold = 10

        lidar_range, lidar_angleInc = self.lidar.getLidarData()
        if (self.obstacle_num == 0):
            for i in range(55, 125):
                if lidar_range[i] < 0.9:
                    if 55 <= i <= 90:
                        stack_L = stack_L + 1
                    elif 91 <= i <= 125:
                        stack_R = stack_R + 1
                if stack_L >= obstacle_threshold / 2:
                    stack_R = 0
                if stack_R >= obstacle_threshold / 2:
                    stack_L = 0
                if stack_L >= obstacle_threshold:
                    self.obstacle_location = 1
                    self.obstacle_num = 1
                if stack_R >= obstacle_threshold:
                    self.obstacle_location = 2
                    self.obstacle_num = 1
            reset_flag = reset_flag + 1
            if reset_flag == 3:
                stack_L, stack_R, reset_flag = 0, 0, 0
        elif (self.obstacle_num != 0):
            for i in lidar_range[85:95]:
                if i < 0.9:
                    stack_C = stack_C + 1
            reset_flag = reset_flag + 1
            if stack_C > obstacle_threshold:
                self.obstacle_location = self.lane_num
            if reset_flag == 3:
                reset_flag, stack_C = 0, 0
        print(self.obstacle_location)


    def obstacleSteering(self):
        time_gap = 1
        steer_angle = 45
        if self.obstacle_location == 1:  # 1차선 장애물
            if time.time() - self.starting_time < time_gap:
                angle, speed = -steer_angle, 5
                print("Evade Mode 1")
            elif time.time() - self.starting_time < time_gap * 2:  # time_gap만큼 반대로 진행
                angle, speed = steer_angle, 5
                print("Evade Mode 2")
            else:
                self.obstacle_location = 0  # steering process end
                # lane_num = 2 추후 2차 장애물 피할 때 추가
                self.obstacle_search_status = True
                angle, speed = 0, 5  # angle,speed

        elif self.obstacle_location == 2:  # 2차선 장애물
            if time.time() - self.starting_time < time_gap:
                angle, speed = steer_angle, 5
            elif time.time() - self.starting_time < time_gap * 2:
                angle, speed = -steer_angle, 5
            elif  time.time() -self.starting_time <time_gap*3 :
                angle, speed = steer_angle-10, 5                
            else:
                self.obstacle_location = 0
                # lane_num = 1 추후 2차 장애물 피할 때 추가
                self.obstacle_search_status = True  # 조향 완전히 종료되면 라이다 다시 작동
                angle, speed = 0, 5
        return angle, speed  # 함수 변수 3개 리턴

    def getObstacleLocation(self):
        return self.obstacle_location
    def resetObstacleLocation(self):
        self.obstacle_location = 0
    def getLaneNum(self):
        return self.lane_num