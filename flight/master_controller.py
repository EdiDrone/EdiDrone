#!/usr/bin/python

'''
Imports
'''
import time
import sys
import os
import numpy
import subprocess
from subprocess import *
from acclHandler import *
from multiprocessing import Process
from threading import Thread, Lock

# The loop time
loop_time=0

# The mutex for accelerometer data
mutex=Lock()

# Delta Constant
# deltaT=

# Infinite loop. 
while True:
	try:
		# Start thread to monitor accelerometer data
		acclHandlerProcess=Process(target=handleAccelerometerSensorData, args=mutex)
		acclHandlerProcess.start()
		acclHandlerProcess.join()

		# Read sensor data. All floats
		# Gyro: Angle of velocity along x, y & z axes. 3 values
		# Accelerometer: Reading along the x, y & z axes of accelerometer data.  3 values
		# TBD: Wait till all data received
		gyroSensorData=None
		while not gyroSensorData:
			gyroSensorData=receiveSensorData()

		#TODO
		print gyroSensorData

		# Unpack gyro sensor data
		gyroData=np.array(sensorData[0], sensorData[1], sensorData[2])

		# Apply Fusion filter to gyro data

	except Exception, e:
		raise e
