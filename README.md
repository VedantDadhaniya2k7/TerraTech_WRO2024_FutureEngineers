WRO 2024 Future Engineers - Team TerraTech (Team code: 1844)
====

This repository contains engineering materials of the self-driven vehicle of Team TerraTech participating in the WRO Future Engineers competition in the season 2024.

## Content

* `Team_Photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `Vehicle_Photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `Video` contains 5 videos that demonstrate the robot's functionality including it completing 3 laps of the Open Round
* `Schematics` schematic diagrams of each electromechanical components (electronic components and motors) used in the vehicle and how they connect to each other and RaspberryPi and also one schematic with the complete circuit connections shown
* `Source_code` contains code of control software for all components which were programmed to participate in the competition
* `3D_models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. It contains the files in both STL and gcode format
* `Component_tests` contains code for testing and making sure each component of the whole robot is functional. Includes test codes for servo motor, rear motors, camera recognition, as well as the vl53l1x distance measurement sensor

## Introduction

### Team Introduction
![image](https://github.com/user-attachments/assets/1ba109ee-f440-4766-902e-4db60402014c)

_Vedant Dadhaniya (Left Side)_

Vedant is a STEM enthusiast who has recurringly been participating in the World Robot Olympiad since 2019 and this is his 4th time participating for it. He began by playing with jigsaw puzzles when he developed interest in LEGO which soon led to introduction to the world of robotics through EV3. Over the years, he has developed many skills in robotics including 3D part designing, electrical wiring, and languages such as Python, C#, HTML/CSS, and Java. He brings his computer vision skills with Raspberry Pi which he had worked on during the COVID Pandemic. He is an avid reader and also likes to play the guitar.

_Devansh Harivallabhdas (Right Side)_ 

Devansh is a Robotics and stem enthusiast from Ahmedabad. It is his first time at World Robot Olympiad. He is 14 and studies in class 9. As a child , Lego building was his favourite activity . He discovered his love for wires , motors and circuits in the Covid lockdown 4 years ago. Ever since he is pursuing it . He also has keen interest in Environmental applications ,Physics and chemistry and a passionate urban farmer. His dream is to create technologies and solutions that can impact the conservation of our environment.

### Reason for choosing the team name _TerraTech_

Terra is Latin for the word earth , since the theme for this WRO is Earth Allies. And Tech as we all know is the shortform of the word technology.




## About the Project

### Initial Working Idea

![image](https://github.com/user-attachments/assets/9cfa5ad9-1015-4733-9848-602d0a3858d4)

### Implementation

We decide to use the Raspberry Pi 4b as the onboard SBC to run the other components 
The Open Round Code has been broken down into chunks and explained below:

```
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
```

The initial part of the code initializes all required libraries and variables

```
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
```

The next part of the code sets up the communication with the sensors (and reassigning their I2C adresses) and motors using the GPIO (General Purpose Input/Output) pins also allocating the timing budgets for the VL53L1X sensors and setting up PWM output for the servo and BO motor.

```
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
```

Now the robot waits for the activating button to be pressed

```
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
```

Once the robot has the green light, it waits for all the sensors to start feeding values over the I2C bus

```
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
```

Finally the robot begins sending the OWM to the BO motor and starts them after having straightened the servo motor. This above section also sets a reference distance the robot will use to find the driving direction.

```
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
```

The robot now moves straight until one of the side sensors show that the distance has increased by at least 20cm. The RPi now sets it as the driving direction and marks it as the sensor of interest

```
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
```

The robot now makes a turn in the detected direction and makes up for the variation in the start placement due to randomisation

```
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
```

The robot now alternates between wall following and turning based on the sensor data and according relays the commands to the servo and BO motor until it has made 12 turns (including the one it already it made before)

```
time.sleep(1.5)
GPIO.output(in2, GPIO.LOW)
```

Now it runs the rear mototrs for just a small while before shutting the off in order to make sure it completely lies in the end area




## How to replicate the robot

First collect all the parts required as mentioned in our Engineering document : https://drive.google.com/file/d/1gBKj8WkkrsB5z0ZKzZZzRHeqWIEaoUqL/view?usp=sharing
and print all the parts as given in the `3D_models` folder of this repository

Now assemble all the 3d printed parts and the electromechanical componentssuch that the basic components and the 3D printed pars look like this:

![Robot design](https://github.com/user-attachments/assets/fd2e6c28-7161-4f5d-b01d-4daa3982a86d)

Then connect the jumper cables as per the following electrical circuit diagram:

![Electrical diagram](https://github.com/user-attachments/assets/33cc4a9a-c106-4b18-8678-df0ee51bfef2)

Before booting the RaspberryPi 4b, flash a 32GB microSD card with a Bookworm image and insert it into the microSD card slot of the RaspberryPi
Now load the Source code onto the raspberry pi using USB to USB communication or a portable data storage device.

Now in the terminal of the RaspberryPi, setup a virtual environment as such:

```> python3 -m venv <give a name to your virtual environment>```

Activate the virtual environment using the following command:

```> source <name of virtual environment>/bin/activate```

Now install the following libraries using ```> pip install <library name>``` :

* opencv-python
* adafruit-circuitpython-vl53l1x
* board
* digitalio
* numpy

Now the robot is ready to run

# References

* https://learn.adafruit.com/adafruit-vl53l1x/python-circuitpython
* https://learn.sparkfun.com/tutorials/qwiic-distance-sensor-vl53l1x-hookup-guide/python-examples
* https://docs.sunfounder.com/projects/raphael-kit/en/latest/appendix/install_the_libraries.html#create-virtual
* https://docs.sunfounder.com/projects/raphael-kit/en/latest/appendix/i2c_configuration.html#i2c-config
* https://www.youtube.com/watch?v=2bganVdLg5Q&t=42s
* https://github.com/World-Robot-Olympiad-Association/wro2022-fe-template
* https://www.youtube.com/playlist?list=PLGs0VKk2DiYyXlbJVaE8y1qr24YldYNDm
