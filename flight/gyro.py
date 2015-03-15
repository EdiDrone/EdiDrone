#!usr/bin/python

import mraa as m
import struct
import sys
import time

gyro=m.I2c(6)
gyro.address(0x69)

# Sensor Data Indexs
AR_X_LSB=0x28
AR_X_MSB=0x29
AR_Y_LSB=0x2a
AR_Y_MSB=0x2b
AR_Z_LSB=0x2c
AR_Z_MSB=0x2d
OUT_TEMP=0x26

# Configuration Parameters
CTRL_REG1=0x20
CTRL_REG2=0x21
CTRL_REG3=0x22
CTRL_REG4=0x23
CTRL_REG5=0x24

# This enables 3 axes at 100 Hz
gyro.writeReg(CTRL_REG1, 0x0F)

# This enables normal mode no high pass
gyro.writeReg(CTRL_REG2, 0x20)

# FIFO and interrupt config
gyro.writeReg(CTRL_REG3, 0x00)

# Data format 
gyro.writeReg(CTRL_REG4, 0x00)

# FIFO and interrupt enable
gyro.writeReg(CTRL_REG5, 0x00)

#gyro.writeReg(0x80, 0x01)


def uint16_2_int(a):
	if a >= 32768:
		a-=65536
	return a


def convertAR(bytes):
	#return struct.unpack('>2B', bytes)
	a =  ((bytes[0] << 8) | bytes[1])
	b = uint16_2_int(a)
	return b



while True:
	gyro.writeByte(0x80)

	i2cBytes=gyro.read(0x40)
	if i2cBytes[0x0F]!=0xD3:
		print 'Error'

	arXData=[i2cBytes[AR_X_MSB],i2cBytes[AR_X_LSB]]
	#print len(i2cBytes[AR_X_MSB])

	#arXData=[chr(i2cBytes[AR_X_MSB]), chr(i2cBytes[AR_X_LSB])]

	#print arXData
	#print len(arXData)

	arYData=[i2cBytes[AR_Y_MSB], i2cBytes[AR_Y_LSB]]

	arZData=[i2cBytes[AR_Z_MSB], i2cBytes[AR_Z_LSB]]


	arXInt=convertAR(arXData)*0.00381469
	arYInt=convertAR(arYData)*0.00381469
	arZInt=convertAR(arZData)*0.00381469

	print str(arXInt) + ' ' + str(arYInt) + ' ' + str(arZInt)
	#time.sleep(0.07)
