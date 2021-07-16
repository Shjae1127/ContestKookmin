from detect_line import Offset
import math

Width = 640
Height = 480

def getSteerAng(pos):
    global Offset
    global Width
    offset = 10
    l = 350                     #350mm
    r = 1250                    #1250mm
    ratio = (float)(845/380)
    lpos = pos[0]
    rpos = pos[1]
    theta1_rad = (float)(320-lpos)/(float)(r) * ratio
    theta2_rad = (float)(rpos-320)/(float)(r) * ratio
    ld = (float)(r * math.cos((theta1_rad + theta2_rad)/2))
    alpha = (theta2_rad-theta1_rad)/2
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha))/(float)(ld)))
    degree = -2.5 * degree
    # center = (lpos+rpos)/2
    # dif = Width/2 - center
    # degree = dif * k
    return degree