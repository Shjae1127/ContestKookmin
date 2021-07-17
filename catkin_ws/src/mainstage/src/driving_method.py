import math



def getSteerAng(pos):
    l = 350                     #350mm
    r = 1250                    #1250mm
    ratio = (float)(845/380)
    center = 320
    
    lpos = pos[0]
    rpos = pos[1]
    theta1_rad = (float)(center-lpos)/(float)(r) * ratio
    theta2_rad = (float)(rpos-center)/(float)(r) * ratio
    ld = (float)(r * math.cos((theta1_rad + theta2_rad)/2))
    alpha = (theta2_rad-theta1_rad)/2
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha))/(float)(ld)))
    degree = -2.5 * degree
    
    return degree