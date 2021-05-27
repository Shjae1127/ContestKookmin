#! /usr/bin/env python

import rospy
import math
import cv2
import time
import rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int32MultiArray

arData = {"DX": 0.0, "DY": 0.0, "DZ": 0.0,
          "AX": 0.0, "AY": 0.0, "AZ": 0.0, "AW": 0.0}

roll, pitch, yaw = 0, 0, 0


def callback(msg):
    global arData

    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w


rospy.init_node('ar_drive')

rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)

motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)

xycar_msg = Int32MultiArray()

angle = 50
speed = 50
goBack = False
goForward = True
while not rospy.is_shutdown():

    (roll, pitch, yaw) = euler_from_quaternion(
        (arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))

    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    img = np.zeros((100, 500, 3))

    img = cv2.line(img, (25, 65), (475, 65), (0, 0, 255), 2)
    img = cv2.line(img, (25, 40), (25, 90), (0, 0, 255), 3)
    img = cv2.line(img, (250, 40), (250, 90), (0, 0, 255), 3)
    img = cv2.line(img, (475, 40), (475, 90), (0, 0, 255), 3)

    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25:
        point = 25

    img = cv2.circle(img, (point, 65), 15, (0, 255, 0), -1)

    distance = math.sqrt(pow(arData["DX"], 2) + pow(arData["DY"], 2))

    cv2.putText(img, str(int(distance))+" pixel", (350, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                + " Yaw:" + str(round(yaw, 1))
    cv2.putText(img, dx_dy_yaw, (20, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))

    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)
    if distance:
        DX = float((arData["DX"]))
        DY = float((arData["DY"]))
        yaw_rad = math.radians(yaw)
        thetaRad = abs(math.atan(DY/DX))
        l = 835/6
        
        if yaw > 0:
            if DX>0:
                slope = yaw_rad + thetaRad
                slope = math.pi - slope
                X = distance * math.cos(slope)
                Y = distance * math.sin(slope)
                alpha = math.atan(abs((Y/(2*X)-math.tan(math.pi/2+yaw_rad))/(1+math.tan(math.pi/2+yaw_rad)*Y/(2*X))))
            elif DX<= 0:
                slope = math.pi - thetaRad + yaw_rad
                slope = math.pi - slope
                X = distance * math.cos(slope)
                Y = distance * math.sin(slope)
                alpha = -math.atan(abs((Y/(2*X)-math.tan(math.pi/2+yaw_rad))/(1+math.tan(math.pi/2+yaw_rad)*Y/(2*X))))
        elif yaw <= 0:
            if DX>0:
                slope = thetaRad + yaw_rad
                slope = math.pi - slope
                X = distance * math.cos(slope)
                Y = distance * math.sin(slope)
                alpha = math.atan(abs((Y/(2*X)-math.tan(math.pi/2+yaw_rad))/(1+math.tan(math.pi/2+yaw_rad)*Y/(2*X))))
            elif DX<=0:
                slope = math.pi - thetaRad + yaw_rad
                slope = math.pi - slope
                X = distance * math.cos(slope)
                Y = distance * math.sin(slope)
                alpha = -math.atan(abs((Y/(2*X)-math.tan(math.pi/2+yaw_rad))/(1+math.tan(math.pi/2+yaw_rad)*Y/(2*X))))
                
        
        ld = math.sqrt(pow(X,2)+pow(Y/2,2))
        angle = round(math.degrees(math.atan(2*l*math.sin(alpha)/ld)))
        
        if(goForward):
            if distance < 100:
                if distance < 70:
                    speed = 0
                    if ((distance < 60) | (int(arData["DX"]) > 8) | (int(arData["DX"]) < -8) | (yaw > 5) | (yaw < -5)):
                        goBack = True
                        goForward = False
            else:
                speed = 50

        if(goBack):
            if distance > 300:
                goBack = False
                goForward = True
            else:
                speed = -50
                angle = -angle
    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)

cv2.destroyAllWindows()
