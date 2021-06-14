<details open="open">
  <p align="center">
  <summary>Table of Contents</summary>
    <ol>
      <li><a href="#예선-과제-1-report">예선 과제 1</a></li>
      <li><a href="#예선-과제-2-report">예선 과제 2</a></li>
      <li><a href="#예선-과제-3-report">예선 과제 3</a></li>
    </ol>
</details>

# 예선 과제
## 예선 과제 1 [(Report)](https://github.com/Shjae1127/ROS/files/6649972/ultra_driver.pdf)  
### 1. 첫 번째 시도(일단 무작정 해보기)  
+ 일정거리 이상 벽이랑 가까워지면 곡선구간이라 판단
  1. 좌, 우측 대각선을 기준으로 어느쪽으로 회전해야할지 판단
  2. 회전해야할 방향으로 조향각에 일정값을 계속 더함(일정값의 기준은 내 마음)
+ 좌우 거리를 기준으로 통로의 중앙으로 가도록 조향
  1. 움직여야할 방향으로 조향각에 일정값을 계속 더함(일정값의 기준은 내 마음)

```
    if xycar_data.data:
        angle = 0
        if xycar_data.data[1] < 300:
            if xycar_data.data[0]>xycar_data.data[2]:
                angle =- 50
            else:
                angle =50
        else:
            if xycar_data.data[7]>xycar_data.data[6]+10:
                angle =-40
            elif xycar_data.data[7]<xycar_data.data[6]-10:
                angle =40
            else:
                angle = 0
        
        
        xycar_msg.data = [angle, 50]
        motor_pub.publish(xycar_msg)

```
+ **문제점**: 통로의 중앙으로 이동하기 위해 조향할 때 조향값이 너무 급격하게 변해 안정적인 주행이 불가능함  
+ **예상 해결 방안**  
  1. PD 제어를 통해 조향각의 안정화
  2. 새로운 알고리즘


### 2. 두 번째 시도(첫 번째 시도 + PD 제어)
+ 일정거리 이상 벽이랑 가까워지면 곡선구간 또는 도착지점 근처라 판단 및 제어
  1. 좌, 우측 대각선 차이가 작으면 도착지점 근처라 판단 -> angle = 0 
  2. .. 크면 곡선구간이라 판단 -> 회전해야할 방향으로 angle = 일정값
+ 직선구간 : 좌우 대각선 거리값을 기준으로 통로의 중앙으로 가도록 조향
  1. 일정값 기준으로 PD 제어 on/off  
  2. on -> 값에 비례(P) 하게 , 이전 값과의 차에 비례(D)하게 조향각 결정


     (이전 값과의 차는 차속 방향을 나타냄 (양수면 차가 중앙차선기준으로 오른쪽방향으로 운동중임을 의미)) 
  3. 조향각의 saturation 을 설정

```
angle ,e, dt, dif, prev = 0,0,0,0,0
kp = 1
kd = 12#13.5
ki = 1
while not rospy.is_shutdown():
    if xycar_data.data:
        diag_dif= float(xycar_data.data[0]-xycar_data.data[2]) # >0 -> to r
        if diag_dif != prev: #diag_dif
            dif = diag_dif - prev #midl

        if xycar_data.data[1] < 300: #-------------------------------------------------------- curve or goal point
            if diag_dif >20: # curve to l
                angle = -50
            elif diag_dif <-20: # curve to r
                angle = 50
            else: # goal point
                angle = 0

            #if midl !=prev_midl:
            #    print(kp*e, kd*dif  , angle,  "curve") 
                
        else: #------------------------------------------------------------------------------- Linear

            if abs(diag_dif) >5: # ------------------PD control
                angle =  int(-kp * diag_dif - kd * dif ) # on
            else:
                angle = 0 # off

            if angle >=50: # ----------------saturation
                angle = 50
            elif angle <=-50:
                angle = -50
            
            #if midl !=prev_midl: # loop is much faster than this -okay
            #    print(kp*e, kd*dif , angle,  "linear")     
                     
        prev = diag_dif #midl
```
 + **문제점**: 직선구간 중앙차선에 가까울 때 제어 X -> 중앙차선 기준으로 경로가 진동
  + **예상 해결 방안**  새로운 알고리즘 
### 3. 세 번째 시도(Pure Pursuit)  
[Reference](https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081)  
다음의 사진 계산과정을 통해 식을 구현함
![Assignment1_PurePursuit](https://user-images.githubusercontent.com/49667821/118361334-ee87e400-b5c5-11eb-82e3-45a2b37272db.jpeg)

```
if (distance.data):
    fl = distance.data[0]
    fm = distance.data[1]
    fr = distance.data[2]
    l = distance.data[7]
    r = distance.data[6]

    theta = math.radians(36.7)
    ld_x = (fr-fl)*math.cos(theta)/2
    ld_y = (fr+fl)*math.sin(theta)/2
    ld = math.sqrt(pow(ld_x, 2)+pow(ld_y, 2))
    alpha = math.atan(ld_x/ld_y)
    L = 164*2/3.33
    angle = math.atan((2*L*math.sin(alpha))/ld)
    angle = math.degrees(angle)
    if fm < 250:
        if fr>fl+10:
            angle = 50
        elif fr<fl-10:
            angle = -50
```  
  + **문제점**: 기본적으로 안정적인 주행이 가능했지만 좌우회전 시에는 기존의 알고리즘이 아니라 강제로 조향을 해주어야 했음  
  + **예상 해결 방안**: 새로운 알고리즘의 도입  

## 예선 과제 2 [(Report)](https://github.com/Shjae1127/ROS/files/6649971/line_find.pdf)
+ Hough Line Transform [Reference](https://docs.opencv.org/3.4.0/d9/db0/tutorial_hough_lines.html)  
  + 다음 사진과 같이 Canny Detector를 이용하여 영상의 edge를 검출한 뒤 Probablistic Hough Line Transform 을 이용하여 좌우 차선의 위치를 구한다. (OpenCV 내장 함수 이용)  
  + 만약 차선을 검출하지 못한다면 이전에 검출했던 차선의 위치를 이용하여 도로의 폭을 계산하여 이를 이용하여 가상의 차선을 만든다.  
  + 영상의 중앙(붉은 사각형)과 좌우 차선의 중앙(가운데 초록 사각형)의 위치를 비교하여 조향각을 찾는다. 이 때, 참고자료(p.24-p.25)를 참고하여 계수를 설정한다.  
![Canny](https://user-images.githubusercontent.com/49667821/119705841-33e4c500-be94-11eb-9c24-78c75c0abfe1.png)

## 예선 과제 3 [(Report)](https://github.com/Shjae1127/ROS/files/6649970/ar_parking.pdf)
### 1. 첫 번째 시도(DX)
  +  AR Tag를 통해 얻은 변수 중 하나인 DX를 이용하여 조향
```
if distance:
    if distance < 100:
        speed = 5
        if distance < 70:
            speed = 0
    else:
        speed = 20
        if ((int(arData["DX"]) != 0)):
            if (int(arData["DX"]) > 0):
                angle = angle + 5
            elif (int(arData["DX"])):
                angle = angle - 5
        else:
            angle = 0
```
+ **문제점**: 위치는 어느정도 만족을 하지만 주차각? 이 너무 큰 오차를 가짐, 이유는 모르겠으나 시작 위치 3에서 AR Tag에 일정 거리 이상 가까워졌을 때 잠시 동안 정보를 리셋시키지 못함
+ **예상 해결 방안**: 새로운 조향 알고리즘 구현, AR Tag에 필요 이상 가까이 갔을 경우 후진하는 알고리즘 추가

### 2. 두 번째 시도(X)  
  + DX, DY를 이용하여 AR Tag를 원점으로 하는 XY 좌표계를 구현하여 그 X값을 이용하여 조향  
<p align="center">
  <img src = "https://user-images.githubusercontent.com/49667821/121926449-f11e5a80-cd78-11eb-8a36-7f16506f0b70.jpg" width="60%">
</p>

+ 다음의 조건을 만족하면 일정거리 후진한다. 이 때의 조향각은 전진할 떄의 조향각에 -를 곱한 값이다.
  +  AR Tag에 필요 이상으로 가까이 갔을 경우
  +  주차되어야할 위치이지만 Yaw의 절댓값이 너무 크거나, DX의 값이 너무 큰 경우  
```
if distance:
    thetaRad = math.atan(float(arData["DY"])/float(arData["DX"]))
    slope = math.tan(math.pi - thetaRad - math.radians(yaw))
    X = math.sqrt(pow(distance, 2)/(pow(slope, 2) + 1))
    Y = X * slope
    if(goForward):
        if distance < 100:
            if distance < 70:
                speed = 0
                if ((distance < 60) | (int(arData["DX"]) > 8) | (int(arData["DX"]) < -8) | (yaw > 5) | (yaw < -5)):
                    goBack = True
                    goForward = False
        else:
            speed = 50
            if (int(arData["DX"]) < 0):
                X = -X
            if(YTemp != 0):
                angle = X
            if Y > YTemp:
                YTemp = Y
                XTemp = X
    if(goBack):
        if distance > 300:
            goBack = False
            goForward = True
        else:
            speed = -50
            if (int(arData["DX"]) < 0):
                X = -X
            if(YTemp != 0):
                angle = -X
            if Y > YTemp:
                YTemp = Y
                XTemp = X
```  
+ **문제점**: 후진이 필요한 상황은 목표점에 거의 도달한 경우인데 이 때의 조향각이 매우 작아 후진의 조향각이 유의미한 변화를 주지 못함.
+ **예상 해결 방안**: 후진 시 조향각을 고정값으로 설정.


### 3. 세 번째 시도(PurePursuit)
+ 두 번째 시도와 마찬가지로 AR Tag를 기준으로 한 XY 좌표계를 설정하고 임의로 설정한 목표점(현재 위치를 (X,Y)라 했을 때 (0,Y/2))을 향한 Pure Pursuit 알고리즘을 구현한다.(계산의 단순화를 위해 Bicycle Model로 가정한다)  
<img src = "https://user-images.githubusercontent.com/49667821/121928071-bddccb00-cd7a-11eb-8ea2-90016ebd6d57.jpg" width="50%"/><img src = "https://user-images.githubusercontent.com/49667821/121928221-e795f200-cd7a-11eb-8c92-ecb520773c3c.jpg" width="50%"/>
<img src = "https://user-images.githubusercontent.com/49667821/121928227-e8c71f00-cd7a-11eb-8ae9-969e0ba81005.jpg" width="50%"/><img src = "https://user-images.githubusercontent.com/49667821/121928229-e95fb580-cd7a-11eb-97cf-9a425277ed0b.jpg" width="50%"/>
+ 변수 설명
  + 계산에 필요한 길이 변수(l)같은 경우 화면에서의 길이와 픽셀값의 길이 비를 이용하여 구한다.
  + beta_rad : 차량이 바라보는 방향과 X축이 이루는 각
  + ld : 차량으로부터 목표점까지의 거리
```
if distance:
    DX = float((arData["DX"]))
    DY = float((arData["DY"]))
    yaw_rad = math.radians(yaw)
    theta_rad = abs(math.atan(DY/DX))
    l = 835/6
    slope = yaw_rad + math.pi/2
    X = distance * math.cos(slope)
    Y = distance * math.sin(slope)

    if yaw > 0:
        if DX>0:
            beta_rad = math.pi - (theta_rad - yaw_rad)
            alpha_rad = math.atan(abs((Y/(2*X)-math.tan(beta_rad))/(1+math.tan(beta_rad)*Y/(2*X))))

        elif DX<= 0:
            beta_rad = theta_rad + yaw_rad
            alpha_rad = -math.atan(abs((Y/(2*X)-math.tan(beta_rad))/(1+math.tan(beta_rad)*Y/(2*X))))

    elif yaw <= 0:
        if DX>0:
            beta_rad = math.pi - theta_rad + yaw_rad
            alpha_rad = math.atan(abs((Y/(2*X)-math.tan(beta_rad))/(1+math.tan(beta_rad)*Y/(2*X))))
        elif DX<=0:
            beta_rad = yaw_rad + theta_rad
            alpha_rad = -math.atan(abs((Y/(2*X)-math.tan(beta_rad))/(1+math.tan(beta_rad)*Y/(2*X))))

    ld = math.sqrt(pow(X,2)+pow(Y/2,2))
    angle = (math.degrees(math.atan(2*l*math.sin(alpha_rad)/ld)))*1.5
```  
+ 후진시 조향각을 특정값(40)으로 고정
```
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
        if yaw_rad<0:
            angle = 40
        elif yaw_rad>0:
            angle = -40
```

