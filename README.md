# 예선 과제
---
### 예선 과제 1
+ 첫 번째 시도(일단 무작정 해보기)  
  + 일정거리 이상 벽이랑 가까워지면 곡선구간이라 판단
    1. 좌, 우측 대각선을 기준으로 어느쪽으로 회전해야할지 판단
    2. 회전해야할 방향으로 조향각에 일정값을 계속 더함(일정값의 기준은 내 마음)
  + 좌우 거리를 기준으로 통로의 중앙으로 가도록 조향
    1. 움직여야할 방향으로 조향각에 일정값을 계속 더함(일정값의 기준은 내 마음)

```
if xycar_data.data:
    if xycar_data.data[1] < 300:
        if xycar_data.data[0]>xycar_data.data[2]:
            angle = angle - 50
            angle = angle - 45
        else:
            angle = angle + 50
            angle = angle + 45
    else:
        if xycar_data.data[7]>xycar_data.data[6]:
            angle = angle - 50
        else :
            angle = angle+50
        if xycar_data.data[7]>xycar_data.data[6]+10:
            angle = angle - 40
        elif xycar_data.data[7]<xycar_data.data[6]-10:
            angle = angle+40
        else:
            angle = 0

```
  + **문제점**: 통로의 중앙으로 이동하기 위해 조향할 때 조향값이 너무 급격하게 변해 안정적인 주행이 불가능함  
  + **예상 해결 방안**  
    1. PD 제어를 통해 조향각의 안정화
    2. 새로운 알고리즘


+ 두 번째 시도(첫 번째 시도 + PD 제어)
  + 일정거리 이상 벽이랑 가까워지면 곡선구간 또는 도착지점 근처라 판단 및 제어
    1. 좌, 우측 대각선 차이가 작으면 도착지점 근처라 판단 -> angle = 0 
    2. .. 크면 곡선구간이라 판단 -> 회전해야할 방향으로 angle = 일정값
  + 직선구간 : 좌우 거리값을 기준으로 통로의 중앙으로 가도록 조향
    1. 일정값 기준으로 PD 제어 on/off  
    2. on -> 값에 비례(P) 하게 , 이전 값과의 차에 비례(D)하게 조향각 결정


       (이전 값과의 차는 차속 방향을 나타냄 (양수면 차가 중앙차선기준으로 오른쪽방향으로 운동중임을 의미)) 
    3. 조향각의 saturation 을 설정

```
    if xycar_data.data:
        midl = float(xycar_data.data[7]-xycar_data.data[6]) # bias to left
        diag_dif= float(xycar_data.data[0]-xycar_data.data[2]) # >0 -> to r
        e = float(des - midl)
        if midl != prev_midl:
            dif = midl - prev_midl

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

            if abs(e) >5: # ------------------PD control
                angle =  int(kp * e - kd * dif ) # on
            else:
                angle = 0 # off

            if angle >=50: # ----------------saturation
                angle = 50
            elif angle <=-50:
                angle = -50 
            
            #if midl !=prev_midl:
            #    print(kp*e, kd*dif , angle,  "linear")     
                         
        prev_midl = midl
```
 + **문제점**: 직선구간 중앙차선에 가까울 때 제어 X -> 중앙차선 기준으로 경로가 진동
  + **예상 해결 방안**  새로운 알고리즘 

+ 세 번째 시도(Pure Pursuit)  
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
        if fr>fl:
            angle = 50
        elif fr<fl:
            angle = -50
```  
  + **문제점**: 기본적으로 안정적인 주행이 가능했지만 좌우회전 시에는 기존의 알고리즘이 아니라 강제로 조향을 해주어야 했음  
  + **예상 해결 방안**: 새로운 알고리즘의 도입  

### 예선 과제 2  
<<보류>>
