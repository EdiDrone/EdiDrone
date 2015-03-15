/*
 * ADX345Accelerometer.cpp
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
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "ADX345Accelerometer.h"
#include <iostream>
#include <math.h>

using namespace std;
#define MAX_BUS 64

//Sensor Data
#define ACC_X_LSB 	0x32
#define ACC_X_MSB 	0x33
#define ACC_Y_LSB 	0x34
#define ACC_Y_MSB 	0x35
#define ACC_Z_LSB 	0x36
#define ACC_Z_MSB 	0x37

//Configuration Parameters
#define TIME_FF     0x29
#define DATA_FORMAT 0x31
#define POWER_CTL   0x2D
#define BW_RATE 	 0x2C

ADX345Accelerometer::ADX345Accelerometer(int bus, int address) {
	I2CBus = bus;
	I2CAddress = address;
	acc = mraa_i2c_init(I2CBus);
	readFullSensorState();
}

//********************************************************************************
// Necessary functions
//********************************************************************************

int ADX345Accelerometer::readFullSensorState(){

	mraa_i2c_address(acc, I2CAddress);

    //cout << "Starting ADX345 I2C sensor state read" << endl;
    //char namebuf[MAX_BUS];

    // In some i2c devices you need to send the first address
    // in write mode and then a stop/start condition is issued. Data bytes are
    // transferred with automatic address increment.
    uint8_t buf[1] = { 0x00 };
    if(mraa_i2c_write(acc, buf, 1) ==1){
    	cout << "Failed to Reset Address in readFullSensorState() " << endl;
    	return 1;
    }

    int numberBytes = ADX345_I2C_BUFFER;
    int bytesRead = mraa_i2c_read(acc, this->dataBuffer, numberBytes);
    if (bytesRead == 0){
    	cout << "Failure to read Byte Stream in readFullSensorState()" << endl;
    	return 2;
    }

    if (this->dataBuffer[0]!=0xe5){
    	cout << "MAJOR FAILURE: DATA WITH ADX345 HAS LOST SYNC!" << endl;
    }

    this->accelerationX = convertAcceleration(ACC_X_MSB, ACC_X_LSB);
    this->accelerationY = convertAcceleration(ACC_Y_MSB, ACC_Y_LSB);
    this->accelerationZ = convertAcceleration(ACC_Z_MSB, ACC_Z_LSB);
    //cout << "Pitch:" << this->getPitch() << "   Roll:" << this->getRoll() <<  endl;
    return 0;
}

int ADX345Accelerometer::convertAcceleration(int msb_reg_addr, int lsb_reg_addr){
	short temp = dataBuffer[msb_reg_addr];
	temp = (temp<<8) | dataBuffer[lsb_reg_addr];
	temp = ~temp + 1;
	return temp;
}

//********************************************************************************
// Extra functions
//********************************************************************************


void ADX345Accelerometer::displayMode(int iterations){

	for(int i=0; i<iterations; i++){
		this->readFullSensorState();
		printf("Rotation (%d, %d, %d)", accelerationX, accelerationY, accelerationZ);
	}
}

uint8_t ADX345Accelerometer::getTIME_FF(){
	this->readFullSensorState();
	char temp = dataBuffer[TIME_FF];
	this->time_ff = (uint8_t) temp;
	return this->time_ff;
}

int ADX345Accelerometer::setTIME_FF(uint8_t desired){
	if(this->writeI2CDeviceByte(TIME_FF, desired)!=0){
		cout << "Failure to update TIME_FF value" << endl;
		return 1;
	}
	return 0;
}

uint8_t ADX345Accelerometer::getDATA_FORMAT(){
	this->readFullSensorState();
	char temp = dataBuffer[DATA_FORMAT];
	this->data_format = (uint8_t) temp;
	return this->data_format;
}

int ADX345Accelerometer::setDATA_FORMAT(uint8_t desired){
	if(this->writeI2CDeviceByte(DATA_FORMAT, desired)!=0){
		cout << "Failure to update DATA_FORMAT value" << endl;
		return 1;
	}
	return 0;
}

uint8_t ADX345Accelerometer::getPOWER_CTL(){
	this->readFullSensorState();
	char temp = dataBuffer[POWER_CTL];
	this->power_ctl = (uint8_t) temp;
	return this->power_ctl;
}

int ADX345Accelerometer::setPOWER_CTL(uint8_t desired){
	if(this->writeI2CDeviceByte(POWER_CTL, desired)!=0){
		cout << "Failure to update POWER_CTL value" << endl;
		return 1;
	}
	return 0;
}

uint8_t ADX345Accelerometer::getBW_RATE(){
	this->readFullSensorState();
	char temp = dataBuffer[BW_RATE];
	this->bw_rate = (uint8_t) temp;
	return this->bw_rate;
}

int ADX345Accelerometer::setBW_RATE(uint8_t desired){
	if(this->writeI2CDeviceByte(BW_RATE, desired)!=0){
		cout << "Failure to update BW_RATE value" << endl;
		return 1;
	}
	return 0;
}

int ADX345Accelerometer::writeI2CDeviceByte(char address, char value){

    cout << "Starting ADX345 I2C sensor state write" << endl;
    //char namebuf[MAX_BUS];

    mraa_i2c_address(acc, I2CAddress);

    uint8_t buffer[2];
    buffer[0] = address;
    buffer[1] = value;
    if ( mraa_i2c_write(acc, buffer, 2) != 2) {
        cout << "Failure to write values to I2C Device address." << endl;
        return(3);
    }
    cout << "Finished ADX345 I2C sensor state write" << endl;
    return 0;
}

ADX345Accelerometer::~ADX345Accelerometer() {
	// TODO Auto-generated destructor stub
}

