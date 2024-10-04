"""
Runs the rear motors for certain amount of time

Can be used for:
 - Checking radius of turning
 - Checking if enough torque is given for given speed
 - etc
"""



"""
============================= IMPORT REQUIRED LIBRARIES =============================
"""
import RPi.GPIO as GPIO           # type: ignore
from time import sleep, time


"""
============================ SETUP ENVIRONMENT VARIABLES ============================
"""
# GPIO Pins
in1         = 23
in2         = 24
en          = 12
servo       = 13
btn         = 27
# Logic variables                                 \             |             /
mot_speed   = 50
btn_stat    = 0             #                      \            |            /
servo_angle = "58"          # Servo Angle - 30 Full Right | 58 center | 86 Full Right
btn_req     = False         # True if button is required to be pressed 
                            # before running the motors, False otherwise


"""
================================== SETUP GPIO PINS ==================================
"""
GPIO.setmode(GPIO.BCM)
# INPUT/OUTPUT Pins Assigned
GPIO.setup(btn   , GPIO.IN)
GPIO.setup(in1   ,GPIO.OUT)
GPIO.setup(in2   ,GPIO.OUT)
GPIO.setup(en    ,GPIO.OUT)
GPIO.setup(servo ,GPIO.OUT)
# Default GPIO Status Assigned
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
# Assign Pins for PWM ( Pulse Width Modulation )
motor  = GPIO.PWM(en    ,1000)
servo1 = GPIO.PWM(servo ,  50) 


"""
=========================== WAIT FOR BUTTON TO BE PRESSED ===========================
"""
start= time()
while btn_req:
    if GPIO.input(btn) == GPIO.HIGH:
        btn_stat = time() - start
        if btn_stat > 1:
            btn_req = False
    else:
        btn_stat = 0
        start = time()
        

"""
================================== TURN ON MOTORS ===================================
"""
servo1.start(100)
motor.start(100)
motor.ChangeDutyCycle(mot_speed)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    
# Set servo angle
servo1.ChangeDutyCycle((float(servo_angle)/18))
sleep(0.5)
servo1.ChangeDutyCycle(0)

# Motor Direction
x='f'


"""
============================== RUN MOTOR FOR n SECONDS ==============================
"""
n = 2
c = time()
while((time()-c) < n):
   
    if x=='r':
        print("run")
        if(temp1==1):
         GPIO.output(in1,GPIO.LOW)
         GPIO.output(in2,GPIO.HIGH)
         print("forward")
         x='z'
        else:
         GPIO.output(in1,GPIO.LOW)
         GPIO.output(in2,GPIO.HIGH)
         print("backward")
         x='z'


    elif x=='s':
        print("stop")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        x='z'

    elif x=='f':
        print("forward")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=1
        x='z'

    elif x=='b':
        print("backward")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=0
        x='z'

    elif x=='l':
        print("low")
        motor.ChangeDutyCycle(25)
        x='z'

    elif x=='m':
        print("medium")
        motor.ChangeDutyCycle(50)
        x='z'

    elif x=='h':
        print("high")
        motor.ChangeDutyCycle(75)
        x='z'
     
   
    elif x=='e':
        GPIO.cleanup()
        motor.stop()
        servo1.stop()
        print("GPIO Clean up")
        break
   
    else:
        print("<<<  wrong data  >>>")
        print("please enter the defined data to continue.....")
    
    
