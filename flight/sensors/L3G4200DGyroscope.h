/*
 * L3G4200DGyroscope.h
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
#ifndef L3G4200DGYROSCOPE_H_
#define L3G4200DGYROSCOPE_H_
#define L3G4200D_I2C_BUFFER 0x40

class L3G4200DGyroscope {
private:
	int I2CBus, I2CAddress;
	uint8_t dataBuffer[L3G4200D_I2C_BUFFER];

	int angularRateX;
	int angularRateY;
	int angularRateZ;
	int temperature;

	mraa_i2c_context gyro;

	uint8_t ctrl_reg1;
	uint8_t ctrl_reg2;
	uint8_t ctrl_reg3;
	uint8_t ctrl_reg4;
	uint8_t ctrl_reg5;

	int  convertAR(int msb_addr, int lsb_addr);
	int  writeI2CDeviceByte(char address, char value);

public:
	L3G4200DGyroscope(int bus, int address);
	void displayMode(int iterations);

	int  readFullSensorState();
	// The following do physical reads and writes of the sensors
	int setCTRL_REG1(uint8_t desired);
	uint8_t getCTRL_REG1();
	int setCTRL_REG2(uint8_t desired);
	uint8_t getCTRL_REG2();
	int setCTRL_REG3(uint8_t desired);
	uint8_t getCTRL_REG3();
	int setCTRL_REG4(uint8_t desired);
    uint8_t getCTRL_REG4();
    int setCTRL_REG5(uint8_t desired);
    uint8_t getCTRL_REG5();

	int getAngularRateX() { return angularRateX; }
	int getAngularRateY() { return angularRateY; }
	int getAngularRateZ() { return angularRateZ; }

	virtual ~L3G4200DGyroscope();
};

#endif /* L3G4200DGYROSCOPE_H_ */






