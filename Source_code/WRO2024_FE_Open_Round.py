import RPi.GPIO as GPIO # type: ignore
import time
import board # type: ignore
import digitalio # type: ignore
import adafruit_vl53l1x # type: ignore

"""
GPIO PINS
"""
btn         = 27
en          = 12
in1         = 23
in2         = 24
servo       = 13
"""
DATA VARIABLES
"""
cur         = 0
lap_counter = 0
turn_dir    = 0
cmd_given   = False
"""
GPIO SETUP
"""
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1   ,GPIO.OUT)
GPIO.setup(in2   ,GPIO.OUT)
GPIO.setup(en    ,GPIO.OUT)
GPIO.setup(servo ,GPIO.OUT)
GPIO.output(in1  ,GPIO.LOW)
GPIO.output(in2  ,GPIO.LOW)
GPIO.setup(btn   , GPIO.IN)
"""
PWM SETUP
"""
mpower = GPIO.PWM(en   ,1000)
servo1 = GPIO.PWM(servo,  50) 
"""
I2C SETUP
"""
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
#sensor 2
shut_2.value        = True
tof_2               = adafruit_vl53l1x.VL53L1X(i2c)
tof_2.distance_mode = 2
tof_2.timing_budget = 100
tof_2.set_address(0x38)
#sensor 3
shut_3.value        = True
tof_3               = adafruit_vl53l1x.VL53L1X(i2c)
tof_3.distance_mode = 2
tof_3.timing_budget = 100
tof_3.set_address(0x48)


"""
WAIT FOR BTN ACTIVATION
"""
start= time.time()
print("Awaiting command")
while not cmd_given:
    if GPIO.input(btn) == GPIO.HIGH:
        cur = time.time() - start
    else:
        cur = 0
        start = time.time()
    if cur > 0.5:
        cmd_given = True
time.sleep(1)
"""
BTN ACTIVATED
"""


"""
START DETECTING
"""
tof_1.start_ranging()
tof_2.start_ranging()
tof_3.start_ranging()
time.sleep(0.1)
d1 = None
d2 = None
d3 = None
while ((d1 == None) or (d2 == None) or (d3 == None)):
	time.sleep(0.1)
	if tof_1.data_ready:
		d1 = tof_1.distance
		print("Sensor 1: ", str( d1 ), end = "         ")
		tof_1.clear_interrupt()
	if tof_2.data_ready:
		d2 = tof_2.distance
		print("Sensor 2: ", str( d2 ), end = "         ")
		tof_2.clear_interrupt()
	if tof_3.data_ready:
		d3 = tof_3.distance
		print("Sensor 3: ", str( d3 ), end = "         ")
		tof_3.clear_interrupt()
	print("")

"""
BEGIN PWM
"""
servo1.start(0)
mpower.start(100)
mpower.ChangeDutyCycle(50)
"""
SET SERVO ANGLE
"""
d = "57"
servo1.ChangeDutyCycle((float(d)/18))
time.sleep(0.5)


if tof_1.data_ready:
	c1 = tof_1.distance
	print("Sensor 1: ", str( c1 ), end = "         ")
	tof_1.clear_interrupt()
if tof_2.data_ready:
	c2 = tof_2.distance
	print("Sensor 2: ", str( c1 ))
	tof_2.clear_interrupt()
time.sleep(0.1)

"""
TURN ON MOTOR
"""
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.HIGH)


while turn_dir == 0:
	time.sleep(0.1)
	if tof_1.data_ready:
		d1 = tof_1.distance
		print("Sensor 1: ", str( d1 ), end = "         ")
		tof_1.clear_interrupt()
	if tof_2.data_ready:
		d2 = tof_2.distance
		print("Sensor 2: ", str( d2 ), end = "         ")
		tof_2.clear_interrupt()
	print("")
	if d1!= None and d1 - c1 > 15:
		turn_dir = 1
		sensor = tof_1
		sleeptime = (1/24)*(c1-25) if c1>25 else 0
	elif d2!= None and d2 - c2 > 15:
		turn_dir = -1
		sensor = tof_2
		sleeptime = (1/24)*(c2-25) if c2>25 else 0
		

servo1.ChangeDutyCycle(0)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
if turn_dir == -1:
	d=80
elif turn_dir == 1:
	d=30
servo1.ChangeDutyCycle((float(d)/18))
time.sleep(0.5)
servo1.ChangeDutyCycle(0)
mpower.ChangeDutyCycle(70)
time.sleep(0.5)
GPIO.output(in2,GPIO.HIGH)
time.sleep(1.55+(turn_dir/20))
GPIO.output(in2,GPIO.LOW)
lap_counter += 1

servo1.ChangeDutyCycle((56/18))
time.sleep(0.3)
servo1.ChangeDutyCycle(0)
mpower.ChangeDutyCycle(60)
time.sleep(0.5)
GPIO.output(in2,GPIO.HIGH)
time.sleep(sleeptime)
GPIO.output(in2,GPIO.LOW)

turning = False
sensor.start_ranging()
time.sleep(0.1)
GPIO.output(in2,GPIO.HIGH)
prev = sensor.distance
sensor.clear_interrupt()
time.sleep(0.1)
while lap_counter < 13:
	time.sleep(0.1)
	if sensor.data_ready:
		dist = sensor.distance
		print("Sensor Of Detected Dir: ", str( dist ), end = '      ')
		sensor.clear_interrupt()
		if dist == None:
			time.sleep(1)
			print("NONE")
			servo1.ChangeDutyCycle((d)/18)
			time.sleep(0.1)
			servo1.ChangeDutyCycle(0)
		elif dist > 36 and prev > 36:
			if turning == False:
				mpower.ChangeDutyCycle(60)
				turning = True
			servo1.ChangeDutyCycle((d)/18)
			time.sleep(0.1)
			servo1.ChangeDutyCycle(0)
			print("Turning")
			prev = dist
		else:
			if turning == True:
				turning = False 
				mpower.ChangeDutyCycle(60)
				lap_counter += 1
			turning = False
			a = 56+(((32-dist)/4 )*6*turn_dir)
			if a > 68:
				a = 68
			elif a < 44:
				a = 44
			servo1.ChangeDutyCycle((a)/18)
			time.sleep(0.1)
			servo1.ChangeDutyCycle(0)
			print("")
			prev = dist
	else:
		print("No data . . . . . . . . . . . . . .. . . . . . ")
		time.sleep(1)
		
time.sleep(1.5)
GPIO.output(in2, GPIO.LOW)
