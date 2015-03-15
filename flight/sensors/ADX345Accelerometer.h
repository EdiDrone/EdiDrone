/*
 * ADX345Accelerometer.h
 *
 *  Created on: Apr 21, 2014
 *      Author: John R. Aleman, College of Engineering, Boston University
 *      Notes: This Definition of a class to interface with the
 *      Analog Devices ADXL345 Digital Accelerometer was created based
 *      on the work of Derek Molloy.
 *      Copyright Derek Molloy, School of Electronic Engineering, Dublin City University
 *      www.eeng.dcu.ie/~molloyd/
 *
 *
 * 		Redistribution and use in source and binary forms, with or without modification,
 * 		are permitted provided that the following conditions are met:
 * 			1. Redistributions of source code must retain the above copyright
 *    			notice, this list of conditions and the following disclaimer.
 * 			2. Redistributions in binary form must reproduce the above copyright
 *    			notice, this list of conditions and the following disclaimer in the
 *    			documentation and/or other materials provided with the distribution.
 *
 * 		THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * 		INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * 		AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * 		BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * 		CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * 		GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * 		HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * 		OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * 		SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "mraa.h"
#ifndef ADX345ACCELEROMETER_H_
#define ADX345ACCELEROMETER_H_
#define ADX345_I2C_BUFFER 0x40

class ADX345Accelerometer {

private:
	int I2CBus, I2CAddress;
	uint8_t dataBuffer[ADX345_I2C_BUFFER];

	int accelerationX;
	int accelerationY;
	int accelerationZ;

	mraa_i2c_context acc;

	uint8_t time_ff;
	uint8_t data_format;
	uint8_t power_ctl;
	uint8_t bw_rate;
	uint8_t mode_config;

	int  convertAcceleration(int msb_addr, int lsb_addr);
	int  writeI2CDeviceByte(char address, char value);

public:
	ADX345Accelerometer(int bus, int address);
	void displayMode(int iterations);

	int  readFullSensorState();
	// The following do physical reads and writes of the sensors
	int setTIME_FF(uint8_t desired);
	uint8_t getTIME_FF();
	int setDATA_FORMAT(uint8_t desired);
	uint8_t getDATA_FORMAT();
	int setPOWER_CTL(uint8_t desired);
	uint8_t getPOWER_CTL();
	int setBW_RATE(uint8_t desired);
    uint8_t getBW_RATE();

	int getAccelerationX() { return accelerationX; }
	int getAccelerationY() { return accelerationY; }
	int getAccelerationZ() { return accelerationZ; }

	virtual ~ADX345Accelerometer();
};

#endif /* ADX345ACCELEROMETER_H_ */
