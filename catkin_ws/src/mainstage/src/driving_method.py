import math



def getSteerAng(pos):

    l = 350                             # vehicle wheelbase    350mm
    r = 1250                           # distance between rear wheel and Datum point    1250mm
    ratio = (float)(845/380)    # ratio of path width to pixel
    lpos = pos[0]
    rpos = pos[1]
    print(lpos, rpos)
    theta1_rad = (float)(320-lpos)/(float)(r) * ratio
    theta2_rad = (float)(rpos-320)/(float)(r) * ratio
    ld = (float)(r * math.cos((theta1_rad + theta2_rad)/2))
    alpha_rad = (theta2_rad-theta1_rad)/2
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha_rad))/(float)(ld)))
    degree = -2.5 * degree

    return degree

def getSteerAng_test(pos, lane_num):

    l = 350                             # vehicle wheelbase    350mm
    r = 1250                           # distance between rear wheel and Datum point    1250mm
    ratio = (float)(845/380)    # ratio of path width to pixel
    lpos = pos[0]
    rpos = pos[1]

    theta1_rad = (float)(320-lpos)/(float)(r) * ratio
    theta2_rad = (float)(rpos-320)/(float)(r) * ratio
    ld_0 = (float)(r * math.cos((theta1_rad + theta2_rad)/2))
    if lane_num == 0:
        ld = ld_0
        alpha_rad = (theta2_rad-theta1_rad)/2
    elif lane_num == 1:
        ld = math.sqrt((pow(1250, 2) * 2 + 2 * pow(ld_0, 2) - pow(((float)(1250) * math.sin((theta1_rad + theta2_rad)/2)), 2))/4)
        alpha_rad = -math.acos(ld_0/(float)(ld))+(theta2_rad-theta1_rad)/2
    
    elif lane_num == 2:
        ld = math.sqrt((pow(1250, 2) * 2 + 2 * pow(ld_0, 2) - pow(((float)(1250) * math.sin((theta1_rad + theta2_rad)/2)), 2))/4)
        alpha_rad = (theta2_rad-theta1_rad)/2 + math.acos(ld_0/(float)(ld))
        
    degree = math.degrees(math.atan((2*(float)(l)*math.sin(alpha_rad))/(float)(ld)))
    degree = -2.5 * degree
    print(lpos, rpos)

    return degree