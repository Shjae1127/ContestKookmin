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

class obstacle:
    #Variable Initialization
    obstacle_search_status = True
    obstacle_num = 0
    obstacle_location = 0  # (0 : None, 1 : Left, 2: Right)
    lidar = lidar_module("obstacle_detect")
    starting_time = 0
    lane_num = 0  # (0 : 중앙, 1 : 왼쪽 차선, 2 : 오른쪽 차선)
    def obstacleDetect(self):

        stack_L, stack_C, stack_R = 0, 0, 0
        reset_flag = 0
        obstacle_threshold = 10

        lidar_range, lidar_angleInc = self.lidar.getLidarData()
        if self.obstacle_search_status & (self.obstacle_num == 0):
            for i in range(70, 110):
                if lidar_range[i] < 0.6:
                    if 70 <= i <= 85:
                        stack_L = stack_L + 1
                    elif 95 <= i <= 110:
                        stack_R = stack_R + 1
                if stack_L >= obstacle_threshold / 2:
                    stack_R = 0
                if stack_R >= obstacle_threshold / 2:
                    stack_L = 0
                if stack_L >= obstacle_threshold:
                    self.obstacle_location = 1
                    self.obstacle_search_status = False
                    obstacle_num = 1
                if stack_R >= obstacle_threshold:
                    self.obstacle_location = 2
                    self.obstacle_search_status = False
                    obstacle_num = 1
            reset_flag = reset_flag + 1
            if reset_flag == 3:
                stack_L, stack_R, reset_flag = 0, 0, 0
        elif self.obstacle_search_status & (self.obstacle_num != 0):
            for i in lidar_range[85:95]:
                if i < 0.6:
                    stack_C = stack_C + 1
            reset_flag = reset_flag + 1
            if stack_C > obstacle_threshold:
                self.obstacle_location = self.lane_num
            if reset_flag == 3:
                reset_flag, stack_C = 0, 0
        print(obstacle_num)


    def obstacleSteering(self):
        time_gap = 1
        if self.obstacle_location == 1:  # 1차선 장애물
            if time.time() - self.obstacle_starting_time < time_gap:
                angle, speed = -50, 5
            elif time.time() - self.obstacle_starting_time < time_gap * 2:  # time_gap만큼 반대로 진행
                angle, speed = 50, 5
            else:
                obstacle_location = 0  # steering process end
                # lane_num = 2 추후 2차 장애물 피할 때 추가
                obstacle_search_status = True
                angle, speed = 0, 5  # angle,speed

        elif self.obstacle_location == 2:  # 2차선 장애물
            if time.time() - self.obstacle_starting_time < time_gap:
                angle, speed = 50, 5
            elif time.time() - self.obstacle_starting_time < time_gap * 2:
                angle, speed = -50, 5
            else:
                self.obstacle_location = 0
                # lane_num = 1 추후 2차 장애물 피할 때 추가
                self.obstacle_search_status = True  # 조향 완전히 종료되면 라이다 다시 작동
                angle, speed = 0, 5
        return angle, speed, obstacle_search_status  # 함수 변수 3개 리턴
    def getObstacleSearchStatus(self):
        return self.obstacle_search_status