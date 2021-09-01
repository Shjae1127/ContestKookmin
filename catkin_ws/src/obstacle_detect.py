#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : obstacle_detect.py
# 수 정 자 : 신홍재, 노현빈
# 생 성 일 : 2021년 07월 16일
# 수 정 일 : 2021년 08월 17일
####################################################################

import rospy, time
from common_ros_cls import ultra_module, lidar_module
from driving_method import getSteerAng


class obstacleQueue:
    def __init__(self):
        self.data = [0]*2
        # ultra_search_stauts = 0, obstacle_location = 0

    def push(self, item):
        self.data.append(item)

    def pop(self):
        return self.data.pop(0)



class obstacle:
    def __init__(self):
        # Variable Initialization
        self.obstacle_search_status = True
        self.obstacle_num = 0
        self.obstacle_location = 0  # (0 : None, 1 : Left, 2: Right)
        self.obstacle_que = obstacleQueue()
        self.lidar = lidar_module("obstacle_detect")
        self.lane_num = 0  # (0 : 중앙, 1 : 왼쪽 차선, 2 : 오른쪽 차선)
        self.ultra_sensor = ""
        self.ultra_data = []
        self.ultra_search_status = 0
        self.ultra_reset = False  # True일 때 초기화
        self.finish_flag = False
        

    def setUltrasonic(self, sonic_list):
        if sonic_list[0]:
            self.ultra_data = sonic_list
            self.left = self.ultra_data[0]
            self.right = self.ultra_data[4]
            # print('sonaleft:{}'.format(self.left),'sonaright:{}'.format(self.right))

    def obstacleDetect(self):
        stack_L, stack_C, stack_R = 0, 0, 0
        reset_flag = 0
        obstacle_threshold = 10
        obstacle_alpha = 1
        lidar_range, lidar_angleInc = self.lidar.getLidarData()
         
        # for i in range(85,95) :
        #     if lidar_range[i]<0.2:
        #         stop_flag +=1
        # if stop_flag >4:
        #     self.stop_drive = True
        #     print("-----------------enmergency!-----------------")
        # stop_flag = 0

        # -----초음파-------------
        if self.ultra_sensor == "left":
            if self.left < 30:  # 초음파에 감지
                self.ultra_search_status = 1  # 초음파 센서에 장애물 계속 걸리는 상태
                self.obstacle_que.push(1)
            else:
                self.ultra_search_status = 0
                self.obstacle_que.push(0)
        elif self.ultra_sensor == "right":
            if self.right < 30:  # 초음파에 감지
                self.ultra_search_status = 1  # 초음파 센서에 장애물 계속 걸리는 상태
                self.obstacle_que.push(1)
            else:
                self.ultra_search_status = 0
                self.obstacle_que.push(0)
        else :
            self.obstacle_que.push(0)
        if self.obstacle_que.pop() - self.ultra_search_status == -1:
            # 초음파 false->true일때만 작동
            self.ultra_reset = True  # reset 트리거 작동할 때만 리셋 시킴
            self.ultra_sensor = ""
        else:
            self.ultra_reset = False

        # -----장애물 인지 -----------
        if self.obstacle_num == 0:
            if self.ultra_reset:  # 초음파 리셋트리거 작동 전까지 계속 작동
                self.obstacle_search_status = True
            if self.obstacle_search_status is False :
                lidar_check = [0]*71
            else :
                lidar_check = [1 if lidar_range[i]<0.8 else 0 for i in range(55,126)]            
            print('L:{}'.format(lidar_check[0:35].count(1)),'R:{}'.format(lidar_check[35:70].count(1)))
                # lidar_check = [0]*41
            # else :
            #     lidar_check = [1 if lidar_range[i]<1.3 else 0 for i in range(70,111)]    
            #     self.obstacle_location = 0           
            #     print('F:{}'.format(lidar_check[0:40].count(1)))
            if lidar_check[0:70].count(1) >obstacle_threshold*2 : #장애물 말고 다른 물체 인식, 초기화시킴
                lidar_check = [0]*71
            elif lidar_check[0:35].count(1) >obstacle_threshold :#왼쪽영역 체크
                self.obstacle_location = 1
                print('detect left')
            elif lidar_check[35:70].count(1) >obstacle_threshold :
                self.obstacle_location = 2
                print('detect right')

            self.obstacle_que.push(self.obstacle_location)# 갱신안하고 큐에 넣음

        elif self.obstacle_num != 0:
            if self.ultra_reset:
                self.obstacle_search_status = True
                if self.obstacle_num == 3:  # 후면 초음파 센서에서 값 상실 할때까지 차선 이동
                    self.obstacle_num = 0
                    self.lane_num = 0
                    self.obstacle_location = 0
                    self.obstacle_que.data = [0]*2
                    lidar_check = []
                    self.finish_flag = True
            if self.obstacle_search_status is False:
                lidar_check = [0]*41
            else :
                lidar_check = [1 if lidar_range[i]<0.8 else 0 for i in range(70,111)]

            print('F:{}'.format(lidar_check[0:41].count(1)))
            if lidar_check[0:41].count(1) >obstacle_threshold*0.9 :
                self.obstacle_location = self.lane_num
                print('obstalce detect!!!-----------------------')
            self.obstacle_que.push(self.obstacle_location)

        # 장애물 인식된 순간
        if self.obstacle_que.pop() != self.obstacle_location:
            self.obstacle_num = self.obstacle_num + 1

            if self.obstacle_location == 1:
                self.lane_num = 2
                self.ultra_sensor = "left"  # 1 차선일 때 좌측 초음파 센서 사용
            elif self.obstacle_location == 2:
                self.lane_num = 1
                self.ultra_sensor = "right"  # 왼쪽 초음파 센서 사용할지 결정
            self.obstacle_search_status = False
        print(self.obstacle_num, self.obstacle_search_status, self.ultra_sensor,self.lane_num)

    def getObstacleLocation(self):
        return self.obstacle_location
    
    def getObstacleNum(self):
        return self.obstacle_num

    def getLaneNum(self):
        return self.lane_num

    def isObstacle(self):
        if self.obstacle_num == 0:
            return False
        else:
            return True

    def getFinishFlag(self) :
        return self.finish_flag

    def resetFinishFlag(self):
        self.finish_flag = False
