"""
Retrieves sensor data and prints it

Can be used for:
 - Checking distance from wall for placement
 - Checking if I2C communication of sensor id functional
 - etc
"""



"""
============================= IMPORT REQUIRED LIBRARIES =============================
"""
import VL53L1X # type: ignore
import RPi.GPIO as GPIO # type: ignore
import time


"""
================================ SETUP VL53L1X SENSOR ===============================
"""
tof1 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof1.open()
tof1.set_timing(66000, 70)
time.sleep(5)


"""
============================ FETCH AND PRINT SENSOR DATA ============================
"""
while True:
	tof1.start_ranging(0)                   # Start ranging, 0 = If custom timing budget is used, 1 = Short Range, 2 = Medium Range, 3 = Long Range
	time.sleep(0.05)
	distance_in_mm = tof1.get_distance()    # Grab the range in mm
	time.sleep(0.05)
	print('Sensor:1',distance_in_mm)
	tof1.stop_ranging()
	time.sleep(0.05)

GPIO.cleanup()
