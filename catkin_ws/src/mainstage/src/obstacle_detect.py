#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : obstacle_detect.py
# 수 정 자 : 신홍재
# 생 성 일 : 2021년 07월 16일
# 수 정 일 : 2021년 
####################################################################

import rospy, time
from common_ros_cls import ultra_module, lidar_module

lidar = lidar_module("obstacle_detect")