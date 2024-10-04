"""
Trying to use shape detection to locate red or green block, show the closest one and print() the no. of pillars detected while autosaving the hsv ranges
(P.S. This version of the code is for RaspberryPi, not PC)
"""



"""
============================= IMPORT REQUIRED LIBRARIES =============================
"""
import cv2
import math
import numpy as np
from picamera2 import Picamera2 # type: ignore
import serial



"""
================================== PI CAMERA SETUP ==================================
"""
p=Picamera2()

WIDTH = 1280   
HEIGHT = 720

p.preview_configuration.main.size=(WIDTH,HEIGHT)
p.preview_configuration.main.format='RGB888'
p.preview_configuration.align()
p.configure('preview')

p.start()




"""
================================= DEFINE FUNCTIONS ==================================
"""

#* empty fuction
def empty(a):
    pass

#* draws shape and bounding box aroung a countour of sufficient area
def getContours(img, imgContour, clr):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    count = 0
    pillars = []

    # draw vert. lines 
    cv2.line(imgContour, (int(WIDTH/3), 0), (int(WIDTH/3), HEIGHT), (0, 255, 255), 1)
    cv2.line(imgContour, (int(2*WIDTH/3), 0), (int(2*WIDTH/3), HEIGHT), (0, 255, 255), 1)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        area_thresh = cv2.getTrackbarPos("AreaThresh", "Parameters")
        if area >= area_thresh:
            count += 1

            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 2)
            
            peri = cv2.arcLength(cnt, True)
            appx = cv2.approxPolyDP(cnt, 0.02*peri, True)
            
            x, y, w, h = cv2.boundingRect(appx) # x: x-coordinate of top left corner of bb, y: y-coordinate of top left corner of bb, w: width , h: height
            coords = [(x, y), (x+w, y), (x, y+h), (x+w, y+h)]
            pillars.append(coords)
            
            drawn = False
            
            for i,j in coords:
                if i < (2*WIDTH/3) and clr == 'green':
                    cv2.rectangle(imgContour, (x, y), (x+w, y+h), (0, 0, 255), 3)
                    drawn = True
                elif i > (WIDTH/3) and clr == 'red':
                    cv2.rectangle(imgContour, (x, y), (x+w, y+h), (0, 0, 255), 3)
                    drawn = True
            if drawn == False:
                cv2.rectangle(imgContour, (x, y), (x+w, y+h), (0, 255, 0), 3)

            # area is inversly proporsional to square of dist
            # at 30cm dist -> area = 64000
            dist = 30/math.sqrt(area/32100)

            cv2.putText(imgContour, "x: " + str(x) + ', y: ' + str(y) + ', w: ' + str(w) + ', h: ' + str(h), (x, y - 70), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(imgContour, "Area" + str(int(area)), (x, y - 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(imgContour, "Dist" + str(int(dist)), (x, y - 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)

    return count, pillars



"""
================================== LOAD MASK DATA ===================================
"""
with open('green_mask.txt', 'r') as gm:
    g_hmin, g_hmax, g_smin, g_smax, g_vmin, g_vmax = gm.read().split(' ')

with open('red_mask.txt', 'r') as rm:
    r_hmin, r_hmax, r_smin, r_smax, r_vmin, r_vmax = rm.read().split(' ')



"""
TRACKBARS SETUP
"""
cv2.namedWindow("Green HSV dials")
cv2.resizeWindow("Green HSV dials", 640, 270)
cv2.createTrackbar("HUE Min", "Green HSV dials", int(g_hmin), 179, empty)
cv2.createTrackbar("HUE Max", "Green HSV dials", int(g_hmax), 179, empty)
cv2.createTrackbar("SAT Min", "Green HSV dials", int(g_smin), 255, empty)
cv2.createTrackbar("SAT Max", "Green HSV dials", int(g_smax), 255, empty)
cv2.createTrackbar("VAL Min", "Green HSV dials", int(g_vmin), 255, empty)
cv2.createTrackbar("VAL Max", "Green HSV dials", int(g_vmax), 255, empty)
cv2.createTrackbar("Start detection", "Green HSV dials", 0, 1, empty)

cv2.namedWindow("Red HSV dials")
cv2.resizeWindow("Red HSV dials", 640, 270)
cv2.createTrackbar("HUE Min", "Red HSV dials", int(r_hmin), 179, empty)
cv2.createTrackbar("HUE Max", "Red HSV dials", int(r_hmax), 179, empty)
cv2.createTrackbar("SAT Min", "Red HSV dials", int(r_smin), 255, empty)
cv2.createTrackbar("SAT Max", "Red HSV dials", int(r_smax), 255, empty)
cv2.createTrackbar("VAL Min", "Red HSV dials", int(r_vmin), 255, empty)
cv2.createTrackbar("VAL Max", "Red HSV dials", int(r_vmax), 255, empty)

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 120)
cv2.createTrackbar("THRESH_2", "Parameters", 150, 255, empty)
cv2.createTrackbar("THRESH_1", "Parameters", 255, 255, empty)
cv2.createTrackbar("AreaThresh", "Parameters", 5000, 30000, empty)



"""
=================================== RUNTIME CODE ====================================
"""
runtime = True
while runtime:
    pillar_cnt = 0
    img = p.capture_array()
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    g_h_min = cv2.getTrackbarPos("HUE Min", "Green HSV dials")
    g_h_max = cv2.getTrackbarPos("HUE Max", "Green HSV dials")
    g_s_min = cv2.getTrackbarPos("SAT Min", "Green HSV dials")
    g_s_max = cv2.getTrackbarPos("SAT Max", "Green HSV dials")
    g_v_min = cv2.getTrackbarPos("VAL Min", "Green HSV dials")
    g_v_max = cv2.getTrackbarPos("VAL Max", "Green HSV dials")
    
    r_h_min = cv2.getTrackbarPos("HUE Min", "Red HSV dials")
    r_h_max = cv2.getTrackbarPos("HUE Max", "Red HSV dials")
    r_s_min = cv2.getTrackbarPos("SAT Min", "Red HSV dials")
    r_s_max = cv2.getTrackbarPos("SAT Max", "Red HSV dials")
    r_v_min = cv2.getTrackbarPos("VAL Min", "Red HSV dials")
    r_v_max = cv2.getTrackbarPos("VAL Max", "Red HSV dials")

    detect = cv2.getTrackbarPos("Start detection", "Green HSV dials")
    
    g_lower = np.array([g_h_min, g_s_min, g_v_min])
    g_upper = np.array([g_h_max, g_s_max, g_v_max])
    
    r_lower = np.array([r_h_min, r_s_min, r_v_min])
    r_upper = np.array([r_h_max, r_s_max, r_v_max])

    g_mask = cv2.inRange(img_hsv, g_lower, g_upper)
    g_result = cv2.bitwise_and(img, img, mask = g_mask)
    
    r_mask = cv2.inRange(img_hsv, r_lower, r_upper)
    r_result = cv2.bitwise_and(img, img, mask = r_mask)

    if detect == 1:
        g_img_Contour = g_result.copy()
        r_img_Contour = r_result.copy()
        
        g_img_blur = cv2.GaussianBlur(g_result, (7,7), 1)
        g_img_gray_with_blur = cv2.cvtColor(g_img_blur, cv2.COLOR_BGR2GRAY)
        
        r_img_blur = cv2.GaussianBlur(r_result, (7,7), 1)
        r_img_gray_with_blur = cv2.cvtColor(r_img_blur, cv2.COLOR_BGR2GRAY)
        
        thresh1 = cv2.getTrackbarPos("THRESH_1", "Parameters")
        thresh2 = cv2.getTrackbarPos("THRESH_2", "Parameters")
        kernel = np.ones((5, 5))
        
        g_imgCanny = cv2.Canny(g_img_gray_with_blur, thresh1, thresh2)
        g_imgDil = cv2.dilate(g_imgCanny, kernel, iterations=1)
        
        r_imgCanny = cv2.Canny(r_img_gray_with_blur, thresh1, thresh2)
        r_imgDil = cv2.dilate(r_imgCanny, kernel, iterations=1)

        cnt1, pillars_g = getContours(g_imgDil, g_img_Contour, 'green')
        cnt2, pillars_r = getContours(r_imgDil, r_img_Contour, 'red')

        g_contour_display = cv2.resize(g_img_Contour, None, None, 0.5, 0.5)
        r_contour_display = cv2.resize(r_img_Contour, None, None, 0.5, 0.5)

        y_min = 0
        closest_p = None
        closest_p_clr = 'None'

        for p in pillars_g+pillars_r:
            if p[3][1] > y_min:
                y_min = p[3][1]
                closest_p = p
                if p in pillars_g:
                    closest_p_clr = 'green'
                else:
                    closest_p_clr = 'red'

        cv2.imshow('g contour', g_contour_display)
        cv2.imshow('r contour', r_contour_display)

        cv2.putText(img, "Count : " + str(int(cnt1+cnt2)), (10, 40), cv2.FONT_HERSHEY_COMPLEX, 0.9, (0, 0, 0), 2)
        cv2.putText(img, "Closest : " + str(closest_p), (10, 75), cv2.FONT_HERSHEY_COMPLEX, 0.9, (0, 0, 0), 2)
        cv2.putText(img, "Closest's Clr : " + str(closest_p_clr), (10, 110), cv2.FONT_HERSHEY_COMPLEX, 0.9, (0, 0, 0), 2)
        if closest_p != None:
            cv2.rectangle(img, closest_p[0], closest_p[3], (0, 0, 0), 3)

    img = cv2.resize(img, None, None, 0.5, 0.5)
    g_mask = cv2.resize(g_mask, None, None, 0.35, 0.35)
    r_mask = cv2.resize(r_mask, None, None, 0.35, 0.35)
    g_result = cv2.resize(g_result, None, None, 0.35, 0.35)
    r_result = cv2.resize(r_result, None, None, 0.35, 0.35)
    cv2.imshow('img', img)
    cv2.imshow('g isolated', g_result)
    cv2.imshow('r isolated', r_result)

    if cv2.waitKey(1) == ord('q'):
        runtime = False



"""
===================================== CLEANUP =====================================
"""
cv2.destroyAllWindows()


"""
================================== SAVE MASK DATA ===================================
"""
with open('green_mask.txt', 'w') as gm:
    gm.write(f'{g_h_min} {g_h_max} {g_s_min} {g_s_max} {g_v_min} {g_v_max}')
    
with open('red_mask.txt', 'w') as rm:
    rm.write(f'{r_h_min} {r_h_max} {r_s_min} {r_s_max} {r_v_min} {r_v_max}')