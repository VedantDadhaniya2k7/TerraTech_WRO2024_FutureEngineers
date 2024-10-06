"""
Trying to use shape detection to locate red or green block, show the closest one and print() the no. of pillars detected while autosaving the hsv ranges
(P.S. This version of the code is for RaspberryPi not PC)
"""



"""
IMPORTS
"""
import cv2
import math
import numpy as np
from picamera2 import Picamera2 # type: ignore
import RPi.GPIO as GPIO # type: ignore
import time
import board # type: ignore
import digitalio # type: ignore
import adafruit_vl53l1x # type: ignore


in1 = 23
in2 = 24
en = 12
servo = 13
SHUT1 = 5
SHUT2 = 6
SHUT3 = 26


"""
GPIO SETUP
"""
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1    , GPIO.OUT)
GPIO.setup(in2    , GPIO.OUT)
GPIO.setup(en     , GPIO.OUT)
GPIO.setup(servo  , GPIO.OUT)
GPIO.output(in1   , GPIO.LOW)
GPIO.output(in2   , GPIO.LOW)


motor  = GPIO.PWM(   en, 1000)
servo1 = GPIO.PWM(servo,   50) 
motor.start(100)
motor.ChangeDutyCycle(50)
servo1.start(0)

i2c = board.I2C()
"""
ToF SHUT PIN SETUP
"""
shut_1 = digitalio.DigitalInOut(board.D5 )
shut_2 = digitalio.DigitalInOut(board.D6 )
shut_3 = digitalio.DigitalInOut(board.D26)
shut_1.switch_to_output(value=False)
shut_2.switch_to_output(value=False)
shut_3.switch_to_output(value=False)
"""
ToF SETUP and ADDRESS REASSIGNMENT
"""
#sensor 1
shut_1.value        = True
tof_1               = adafruit_vl53l1x.VL53L1X(i2c)
tof_1.distance_mode = 2
tof_1.timing_budget = 100
tof_1.set_address(0x28)
tof_1.start_ranging()
#sensor 2
shut_2.value        = True
tof_2               = adafruit_vl53l1x.VL53L1X(i2c)
tof_2.distance_mode = 2
tof_2.timing_budget = 100
tof_2.set_address(0x38)
tof_2.start_ranging()
#sensor 3
shut_3.value        = True
tof_3               = adafruit_vl53l1x.VL53L1X(i2c)
tof_3.distance_mode = 2
tof_3.timing_budget = 100
tof_3.set_address(0x48)
tof_3.start_ranging()



"""
Pi Cam Setup
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
FUNCTIONS
"""

#* empty fuction
def empty(a):
    pass

#* draws shape and bounding box aroung a countour of sufficient area
def getContours(img, imgContour, clr):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    count = 0
    pillars = []
    correct_directions = []

    # draw vert. lines 
    cv2.line(imgContour, (int(WIDTH/3), 0), (int(WIDTH/3), HEIGHT), (0, 255, 255), 1)
    cv2.line(imgContour, (int(2*WIDTH/3), 0), (int(2*WIDTH/3), HEIGHT), (0, 255, 255), 1)
    
    cv2.line(imgContour, ( 291, 720), (679, 419), (0, 255, 255), 1)
    cv2.line(imgContour, (1110, 720), (679, 419), (0, 255, 255), 1)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        area_thresh = cv2.getTrackbarPos("AreaThresh", "Parameters")
        if area >= area_thresh:
            count += 1

            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 2)
            
            peri = cv2.arcLength(cnt, True)
            appx = cv2.approxPolyDP(cnt, 0.02*peri, True)
            
            x, y, w, h = cv2.boundingRect(appx) # x: x-coordinate of top left corner of bb, y: y-coordinate of top left corner of bb, w: width , h: height
                
            if y > 355:
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
                
                asdfg = (0, 0, 255) if (((((137*(x+w))+(177*(y+h))-167362>0) or (x+w > 679.6))and(clr == 'red')) or ((((-192*(x+w))+(275*(y+h))+15120>0) or (x+w < 679.6))and(clr=='green'))) else (0, 255, 0)
                correct_dir = True if asdfg == (0, 255, 0) else False
                correct_directions.append(correct_dir)
                cv2.putText(imgContour, "Area" + str(int(area)), (x, y - 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, asdfg, 2)
                cv2.putText(imgContour, "Dist" + str(int(dist)), (x, y - 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, asdfg, 2)

    return count, pillars, correct_directions



"""
LOAD MASK DATA
"""
with open('green_mask.txt', 'r') as gm:
    g_hmin, g_hmax, g_smin, g_smax, g_vmin, g_vmax = gm.read().split(' ')

with open('red_mask.txt', 'r') as rm:
    r_hmin, r_hmax, r_smin, r_smax, r_vmin, r_vmax = rm.read().split(' ')

first_round = True
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
cv2.createTrackbar("Start detection", "Green HSV dials", 1, 1, empty)

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
cv2.createTrackbar("THRESH_2", "Parameters", 10, 255, empty)
cv2.createTrackbar("THRESH_1", "Parameters", 20, 255, empty)
cv2.createTrackbar("AreaThresh", "Parameters", 650, 5000, empty)



"""
RUNTIME CODE
"""
hrw = False
ref_distance = 40 
wall_follow_start = False
runtime = True
mov_dir_last = 0
while runtime:
    movement_dir = 0
    start_motor = 0
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
    
    r_lower1 = np.array([0, r_s_min, r_v_min])
    r_upper1 = np.array([r_h_min, r_s_max, r_v_max])
    
    r_lower2 = np.array([r_h_max, r_s_min, r_v_min])
    r_upper2 = np.array([179, r_s_max, r_v_max])

    g_mask = cv2.inRange(img_hsv, g_lower, g_upper)
    g_result = cv2.bitwise_and(img, img, mask = g_mask)
    
    r_mask1 = cv2.inRange(img_hsv, r_lower1, r_upper1)
    r_result1 = cv2.bitwise_and(img, img, mask = r_mask1)
    r_mask2 = cv2.inRange(img_hsv, r_lower2, r_upper2)
    r_result2 = cv2.bitwise_and(img, img, mask = r_mask2)

    r_result = cv2.bitwise_or(r_result1, r_result2)
    
    if True:
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

        cnt1, pillars_g, cd_g = getContours(g_imgDil, g_img_Contour, 'green')
        cnt2, pillars_r, cd_r = getContours(r_imgDil, r_img_Contour, 'red')

        g_contour_display = cv2.resize(g_img_Contour, None, None, 0.5, 0.5)
        r_contour_display = cv2.resize(r_img_Contour, None, None, 0.5, 0.5)

        y_min = 0
        closest_p = None
        closest_p_clr = 'None'

        for i, pil in enumerate(pillars_g+pillars_r):
            if pil[3][1] > y_min:
                y_min = pil[3][1]
                closest_p = pil
                if pil in pillars_g:
                    closest_p_clr = 'green'
                    movement_dir = -1
                    mov_dir_last = -1
                    if i < len(cd_g) and not cd_g[i] :
                        start_motor = 1
                else:
                    closest_p_clr = 'red'
                    movement_dir = 1
                    mov_dir_last = 1
                    if i < len(cd_r) and not cd_r[i] :
                        start_motor = 1

        cv2.imshow('g contour', g_contour_display)
        cv2.imshow('r contour', r_contour_display)

        cv2.putText(img, "Count : " + str(int(cnt1+cnt2)), (10, 40), cv2.FONT_HERSHEY_COMPLEX, 0.9, (0, 0, 0), 2)
        cv2.putText(img, "Closest : " + str(closest_p), (10, 75), cv2.FONT_HERSHEY_COMPLEX, 0.9, (0, 0, 0), 2)
        cv2.putText(img, "Closest's Clr : " + str(closest_p_clr), (10, 110), cv2.FONT_HERSHEY_COMPLEX, 0.9, (0, 0, 0), 2)
        if closest_p != None:
            cv2.rectangle(img, closest_p[0], closest_p[3], (0, 0, 0), 3)

    img = cv2.resize(img, None, None, 0.5, 0.5)
    g_result = cv2.resize(g_result, None, None, 0.35, 0.35)
    r_result = cv2.resize(r_result, None, None, 0.35, 0.35)
    cv2.imshow('img', img)
    cv2.imshow('g isolated', g_result)
    cv2.imshow('r isolated', r_result)
    tof_3.start_ranging()
    time.sleep(0.5)
    if tof_3.data_ready:
        # check for front clearance
        da = tof_3.distance
        print("Sensor 2: ", str( da ), end = "         ")
        dw = da if da is not None else dw
        tof_3.clear_interrupt()
        MIN_DST = 20 if movement_dir == 1 else 80
        MIN_DST = 80 if movement_dir == -1 else 100
        if (dw is not None and (cnt1+cnt2) >= 0 and dw > MIN_DST) or ((cnt1+cnt2)>0 ):
            if start_motor == 1:
                servo1.ChangeDutyCycle(((56+(movement_dir*7))/18))
                time.sleep(0.1)
                servo1.ChangeDutyCycle(0)
                wall_follow_start = False
                GPIO.output(in2, GPIO.HIGH)
            elif start_motor == 0:
                if True:
                    if mov_dir_last == 1:
                        if tof_2.data_ready:
                            d = tof_2.distance
                            print("Sensor 2: ", str( d ), end = "         ")
                            tof_2.clear_interrupt()
                            time.sleep(0.05)
                        ref_distance = d
                        if d is not None and d < 17:
                            a = 56 - ((18-d)*7/3)
                        # elif d > 20:
                            # a = 56 - ((15-d)*7/3)
                        else:
                            a = 56
                        servo1.ChangeDutyCycle((a/18))
                        time.sleep(0.1)
                        servo1.ChangeDutyCycle(0)
                    elif mov_dir_last == -1:
                        d = None
                        if tof_1.data_ready:
                            d = tof_1.distance
                            print("Sensor 2: ", str( d ), end = "         ")
                            tof_1.clear_interrupt()
                            time.sleep(0.05)
                        ref_distance = d
                        if d is not None and d < 17:
                            a = 56 + ((18-d)*7/3)
                        # elif d > 20:
                            # a = 56 + ((15-d)*7/3)
                        else:
                            a = 56
                        servo1.ChangeDutyCycle((a/18))
                        time.sleep(0.1)
                        servo1.ChangeDutyCycle(0)
                    else:
                        de = 0
                        a = 56 
                        servo1.ChangeDutyCycle((a/18))
                        time.sleep(0.1)
                        servo1.ChangeDutyCycle(0)
                        
                    GPIO.output(in2, GPIO.HIGH)
        elif dw is not None and dw<MIN_DST:
            if mov_dir_last == 1:
                d=100
                time.sleep(5)
                GPIO.output(in2, GPIO.HIGH)
                while d>30:
                    tof_3.start_ranging()
                    time.sleep(0.3)
                    if tof_3.data_ready:
                        da = tof_3.distance
                        print("Sensor 2: ", str( da ), end = "         ")
                        d = da if da is not None else d
                        tof_3.clear_interrupt()
                GPIO.output(in2, GPIO.LOW)
                tof_1.start_ranging()
                tof_2.start_ranging()
                time.sleep(0.3)
                if tof_1.data_ready:
                    da = tof_1.distance
                    print("Sensor 2: ", str( da ), end = "         ")
                    d1 = da if da is not None else d
                    tof_1.clear_interrupt()
                if tof_2.data_ready:
                    da = tof_2.distance
                    print("Sensor 2: ", str( da ), end = "         ")
                    d2 = da if da is not None else d
                    tof_2.clear_interrupt()
                if d1>150:
                    driving_direction = -1
                else:
                    driving_direction = 1
                
                GPIO.output(in1, GPIO.HIGH)
                time.sleep(5)
                
            elif mov_dir_last == -1:
                pass
        else:
            GPIO.output(in2, GPIO.LOW)
    
            
    
    if cv2.waitKey(1) == ord('q'):
        runtime = False



"""
CLEANUP
"""
GPIO.cleanup()
cv2.destroyAllWindows()

with open('green_mask.txt', 'w') as gm:
    gm.write(f'{g_h_min} {g_h_max} {g_s_min} {g_s_max} {g_v_min} {g_v_max}')
    
with open('red_mask.txt', 'w') as rm:
    rm.write(f'{r_h_min} {r_h_max} {r_s_min} {r_s_max} {r_v_min} {r_v_max}')
