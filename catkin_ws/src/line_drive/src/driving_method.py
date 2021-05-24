# -*- coding: utf-8 -*-

# from line_drive import Height, Width
from detect_line import Offset
import math

def purePursuit(pos):
    global Offset
    # global Width, Height
    lpos = pos[0]
    rpos = pos[1]
    Width, Height = 640, 480
    center = (lpos+rpos)/2
    l = 377                                             #1:10 scale rc car
    pathwidth = 845                          #대회 규격
    scale = float(rpos-lpos)/float(pathwidth)
    l = float(l) * scale
    ld = math.sqrt(pow((-center+Width/2),2)+pow((-Offset+Height),2))
    ld_x = -center+Width/2
    ld_y = -Offset + Height
    # alpha = math.atan(ld_x/ld_y)
    alpha = math.atan(float(ld_x)/float(ld_y))
    degree = math.degrees(math.atan((2*l*math.sin(alpha))/float(ld)))
    angle = 2.5 * degree
    angle = round(angle)
    return degree
    
