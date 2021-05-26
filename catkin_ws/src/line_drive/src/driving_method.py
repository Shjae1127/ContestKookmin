from detect_line import Offset

Width = 640
Height = 480

def getSteerAng(pos):
    global Offset
    global Width
    
    k = 0.4                     # -50<degree<50
    lpos = pos[0]
    rpos = pos[1]
    center = (lpos+rpos)/2
    dif = Width/2 - center
    degree = dif * k
    if degree < 50 and degree>-50:
        print(degree)
    elif degree > 50 or degree < -50:
        print("---------------------------")
    return degree