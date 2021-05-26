# -*- coding: utf-8 -*-

# from line_drive import Height, Width
from detect_line import Offset
import math

def getSteerAng(pos):
    global Offset
    
    k = 0.4
    # global Width, Height
    lpos = pos[0]
    rpos = pos[1]
    Width, Height = 640, 480
    center = (lpos+rpos)/2
    dif = Width/2 - center
    degree = dif * k
    print(degree)
    return degree
    
