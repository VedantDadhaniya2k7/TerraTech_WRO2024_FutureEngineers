"""
============================= IMPORT REQUIRED LIBRARIES =============================
"""
import RPi.GPIO as GPIO # type: ignore
import time


"""
================================== SETUP GPIO PINS ==================================
"""
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(13,GPIO.OUT)
# Setup PWM
servo1 = GPIO.PWM(13,50)


"""
=================================== TURN ON SERVO ===================================
"""
servo1.start(0)
print ("Waiting for 2 seconds")
time.sleep(2)


"""
========================== TURN SERVO TO GIVEN INPUT ANGLE ==========================
"""
while True:
	try:
		x = float(input("Angle:"))
        
		if True:
			servo1.ChangeDutyCycle((float(x)/18))
			time.sleep(0.05)
			servo1.ChangeDutyCycle(0)
        
	except Exception as e:
		print(e)
		break 


"""
===================================== CLEANUP =====================================
"""
servo1.stop()
GPIO.cleanup()
print ("Goodbye")
