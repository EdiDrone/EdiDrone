/*
 * L3G4200DGyroscope.cpp
 *
 *  Created on: Apr 21, 2014
 *      Author: John R. Aleman, College of Engineering, Boston University
 *      Notes: This Definition of a class to interface with the
 *      STMicroelectronics L3g4200D Digital output gyroscope was created based
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
#include "L3G4200DGyroscope.h"
#include <iostream>
#include <math.h>

using namespace std;
#define MAX_BUS 64

//Sensor Data
#define AR_X_LSB 	0x28
#define AR_X_MSB 	0x29
#define AR_Y_LSB 	0x2a
#define AR_Y_MSB 	0x2b
#define AR_Z_LSB 	0x2c
#define AR_Z_MSB 	0x2d
#define OUT_TEMP 	0x26

//Configuration Parameters
#define CTRL_REG1  0x20
#define CTRL_REG2  0x21
#define CTRL_REG3  0x22
#define CTRL_REG4	0x23
#define CTRL_REG5  0x24


L3G4200DGyroscope::L3G4200DGyroscope(int bus, int address) {
	mraa_init();
	I2CBus = bus;
	I2CAddress = address;
	//mraa_i2c_context gyro;
	gyro = mraa_i2c_init(I2CBus);
	readFullSensorState();
}

//********************************************************************************
// Necessary functions
//********************************************************************************

int L3G4200DGyroscope::readFullSensorState(){
    //char namebuf[MAX_BUS];


    mraa_i2c_address(gyro, I2CAddress);

    // In some i2c devices you need to send the first address
    // in write mode and then a stop/start condition is issued. Data bytes are
    // transferred with automatic address increment. For the L3G4200D if reading
    // multiple addresses at the same time the start condition must be set to
    // address 0x80. Datasheet page 23
    uint8_t buf[1] = { 0x80 };
    if(mraa_i2c_write(gyro, buf, 1) ==1){
    	cout << "Failed to Reset Address in readFullSensorState() " << endl;
    	return 1;
    }

    int numberBytes = L3G4200D_I2C_BUFFER;
    int bytesRead = mraa_i2c_read(gyro, this->dataBuffer, numberBytes);
    if (bytesRead == 0){
      	cout << "Failure to read Byte Stream in readFullSensorState()" << endl;
      	return 2;
    }
    if (this->dataBuffer[0x0f]!=0xd3){
    	cout << "MAJOR FAILURE: DATA WITH L3G4200D HAS LOST SYNC!" << endl;
    }
    //mraa_i2c_stop(gyro);

    this->angularRateX = convertAR(AR_X_MSB, AR_X_LSB);
    this->angularRateY = convertAR(AR_Y_MSB, AR_Y_LSB);
    this->angularRateZ = convertAR(AR_Z_MSB, AR_Z_LSB);
    return 0;
}

int L3G4200DGyroscope::convertAR(int msb_reg_addr, int lsb_reg_addr){
	short temp = dataBuffer[msb_reg_addr];
	temp = (temp<<8) | dataBuffer[lsb_reg_addr];
	temp = ~temp + 1;
	return temp;
}

//********************************************************************************
// Extra functions
//********************************************************************************


void L3G4200DGyroscope::displayMode(int iterations){

	for(int i=0; i<iterations; i++){
		this->readFullSensorState();
		printf("Rotation (%d, %d, %d)", angularRateX, angularRateY, angularRateZ);
	}
}

uint8_t L3G4200DGyroscope::getCTRL_REG1(){
	this->readFullSensorState();
	char temp = dataBuffer[CTRL_REG1];
	this->ctrl_reg1 = (uint8_t) temp;
	return this->ctrl_reg1;
}

int L3G4200DGyroscope::setCTRL_REG1(uint8_t desired){
	if(this->writeI2CDeviceByte(CTRL_REG1, desired)!=0){
		cout << "Failure to update CTRL_REG1 value" << endl;
		return 1;
	}
	return 0;
}

uint8_t L3G4200DGyroscope::getCTRL_REG2(){
	this->readFullSensorState();
	char temp = dataBuffer[CTRL_REG2];
	this->ctrl_reg2 = (uint8_t) temp;
	return this->ctrl_reg2;
}

int L3G4200DGyroscope::setCTRL_REG2(uint8_t desired){
	if(this->writeI2CDeviceByte(CTRL_REG2, desired)!=0){
		cout << "Failure to update CTRL_REG2 value" << endl;
		return 1;
	}
	return 0;
}
uint8_t L3G4200DGyroscope::getCTRL_REG3(){
	this->readFullSensorState();
	char temp = dataBuffer[CTRL_REG3];
	this->ctrl_reg3 = (uint8_t) temp;
	return this->ctrl_reg3;
}

int L3G4200DGyroscope::setCTRL_REG3(uint8_t desired){
	if(this->writeI2CDeviceByte(CTRL_REG3, desired)!=0){
		cout << "Failure to update CTRL_REG3 value" << endl;
		return 1;
	}
	return 0;
}
uint8_t L3G4200DGyroscope::getCTRL_REG4(){
	this->readFullSensorState();
	char temp = dataBuffer[CTRL_REG4];
	this->ctrl_reg4 = (uint8_t) temp;
	return this->ctrl_reg4;
}

int L3G4200DGyroscope::setCTRL_REG4(uint8_t desired){
	if(this->writeI2CDeviceByte(CTRL_REG4, desired)!=0){
		cout << "Failure to update CTRL_REG4 value" << endl;
		return 1;
	}
	return 0;
}
uint8_t L3G4200DGyroscope::getCTRL_REG5(){
	this->readFullSensorState();
	char temp = dataBuffer[CTRL_REG5];
	this->ctrl_reg5 = (uint8_t) temp;
	return this->ctrl_reg5;
}

int L3G4200DGyroscope::setCTRL_REG5(uint8_t desired){
	if(this->writeI2CDeviceByte(CTRL_REG5, desired)!=0){
		cout << "Failure to update CTRL_REG5 value" << endl;
		return 1;
	}
	return 0;
}


int L3G4200DGyroscope::writeI2CDeviceByte(char address, char value){

    cout << "Starting ADX345 I2C sensor state write" << endl;
    //char namebuf[MAX_BUS];

    //mraa_i2c_context gyro;
    //gyro = mraa_i2c_init(0);
    mraa_i2c_address(gyro, I2CAddress);

    uint8_t buffer[2];
    buffer[0] = address;
    buffer[1] = value;
    if ( mraa_i2c_write(gyro, buffer, 2) != 2) {
            cout << "Failure to write values to I2C Device address." << endl;
            return(3);
        }
    cout << "Finished ADX345 I2C sensor state write" << endl;
    return 0;
}

L3G4200DGyroscope::~L3G4200DGyroscope() {
	// TODO Auto-generated destructor stub
}



