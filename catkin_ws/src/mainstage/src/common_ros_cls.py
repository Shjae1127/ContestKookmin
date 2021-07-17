#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : common_ros_cls.py
# 모 델 명 : ALL
# 작 성 자 : 자이트론
# 수 정 자 : 신홍재
# 생 성 일 : 2021년 03월 25일
# 수 정 일 : 2021년 03월 30일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

class ultra_module:

    ult_data = []

    def __init__(self, node_name):
        self.subscriber_init()

    def subscriber_init(self):
        rospy.Subscriber('/xycar_ultrasonic',             Int32MultiArray, self.ros_ult_callback)

    def rospy_is_shutdown(self):
        return rospy.is_shutdown()

    def ros_ult_callback(self, data):
        self.ult_data = data.data

    def get_ultrasonic_data(self):
        return self.ult_data



class lidar_module:
    lidar_data = []

    lidar_increment = 0

    def __init__(self, node_name):
        self.subscriberInit()

    def subscriberInit(self):
        rospy.Subscriber('/scan',                         LaserScan,       self.rosLidarCallback)

    def rospyIsShutdown(self):
        return rospy.is_shutdown()
        
    def rosLidarCallback(self, data):
        self.lidar_data = data.ranges
        self.lidar_increment = data.angle_increment

    def getLidarData(self):
        return self.lidar_data, self.lidar_increment

