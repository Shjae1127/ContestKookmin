import time
from common_ros_cls import ros_module

r = ros_module("sensor_drive")


def parking():

    ultra_data = r.get_ultrasonic_data()
    left = ultra_data[0:1]
    right = ultra_data[4:5]
    backright = ultra_data[6:7]
    back = ultra_data[7:8]
    backleft = ultra_data[8:9]

    return ultra_data



