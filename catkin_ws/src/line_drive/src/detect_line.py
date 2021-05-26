import cv2
import numpy as np

Offset = 320
lpos, rpos = 125, 560
pathwidth = rpos - lpos

def detectLine(frame):
    global lpos, rpos, pathwidth
    #####  Variable Initailization  #####
    src = frame
    w, h = 150, 50
    threshold, minLineLength, maxLineGap = 50, 5, 100
    num_L, num_R = 0, 0
    line_L_X1, line_L_Y1 = 0, 0
    line_L_X2, line_L_Y2 = 0, 0
    line_R_X1, line_R_Y1 = 0, 0
    line_R_X2, line_R_Y2 = 0, 0
    
    ##### Hough Line Transform #####
    frame = cv2.Canny(frame, 400, 200, None, 3)     # detect the edges of image by using Canny detector
    w = 100
    x_L = lpos-w/2
    if x_L < 0:
        x_L = 0
    x_R = rpos-w/2
    ### Set ROI tracing lpos, and rpos  ###
    roi_L = frame[Offset:Offset+h, x_L:x_L+w]       
    roi_R = frame[Offset:Offset+h, x_R:x_R+w]
    # frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    linesP_L = cv2.HoughLinesP(
        roi_L, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)      #run Probablistic Hough Line Transform for detect the line of left side
    linesP_R = cv2.HoughLinesP(
        roi_R, 1, np.pi / 180, threshold, None, minLineLength, maxLineGap)      #run Probablistic Hough Line Transform for detect the line of right side
    
    ##########################################################################################
    # case1 (num_L!=0 & num_R!=0): both left, right line detected  >>  save pathwidth                                                                                                 #
    # case2,3 ((num_L!=0 & num_R==0) | (num_L==0 & num_R!=0)): ONLY one side line detected >>  set imaginary line using pathwidth   #
    # case4 (num_L==0 & num_R==0): do nothing                                                                                                                                                                        #
    ##########################################################################################
    if linesP_L is not None:
        for i in range(0, len(linesP_L)):
            l = linesP_L[i][0]
            line_L_X1 = line_L_X1 + x_L+l[0]
            line_L_Y1 = line_L_Y1 + Offset+l[1]
            line_L_X2 = line_L_X2 + x_L+l[2]
            line_L_Y2 = line_L_Y2 + Offset+l[3]
            num_L = num_L + 1

    if linesP_R is not None:
        for i in range(0, len(linesP_R)):
            l = linesP_R[i][0]
            line_R_X1 = line_R_X1 + x_R+l[0]
            line_R_Y1 = line_R_Y1 + Offset+l[1]
            line_R_X2 = line_R_X2 + x_R+l[2]
            line_R_Y2 = line_R_Y2 + Offset+l[3]
            num_R = num_R + 1
    if (num_L > 0)&(num_L<3):
        line_L_X1, line_L_Y1 = line_L_X1/num_L,  line_L_Y1/num_L
        line_L_X2, line_L_Y2 = line_L_X2/num_L, line_L_Y2/num_L
        # cv2.line(src, (line_L_X1, line_L_Y1), (line_L_X2,
                                            #    line_L_Y2), (0, 0, 255), 3, cv2.LINE_AA)
        lpos = (Offset + h/2-line_L_Y1)*(line_L_X1-line_L_X2) / \
            (line_L_Y1-line_L_Y2)+line_L_X1

    if (num_R > 0)&(num_R<3):
        line_R_X1, line_R_Y1 = line_R_X1/num_R,  line_R_Y1/num_R
        line_R_X2, line_R_Y2 = line_R_X2/num_R, line_R_Y2/num_R

        # cv2.line(src, (line_R_X1, line_R_Y1), (line_R_X2,
                                            #    line_R_Y2), (0, 0, 255), 3, cv2.LINE_AA)
        rpos = (Offset + h/2-line_R_Y1)*(line_R_X1-line_R_X2) / \
            (line_R_Y1-line_R_Y2)+line_R_X1
    if (num_L != 0) & (num_R != 0):
        pathwidth = rpos - lpos
    elif (num_L != 0) & (num_R == 0):
        rpos = lpos + pathwidth
    elif (num_L == 0) & (num_R != 0):
        lpos = rpos - pathwidth

    return src, lpos, rpos
