# 예선 과제
---
### 예선 과제 1
+ 첫 번째 시도(일단 무작정 해보기)  
  + 일정거리 이상 벽이랑 가까워지면 회전이라 판단
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
    1. PID 제어를 통해 조향각의 안정화
    2. 새로운 알고리즘
+ 두 번째 시도(첫 번째 시도 + PID 제어)
```
//현진이가 수정
```

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
