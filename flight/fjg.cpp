//============================================================================
// Name        : AccTest.cpp
// Author      : John R Aleman
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "sensors/ADX345Accelerometer.h"
#include "sensors/L3G4200DGyroscope.h"

using namespace std;

int main() {

	cout << "Servo set test " << endl;


	/* Accelerometer Initialization */
	ADX345Accelerometer Accelerometer(6,0x53); //0x53 is the Accelerometer address
	if (Accelerometer.getTIME_FF()!=0x14){
		Accelerometer.setTIME_FF(0x14); //100ms free fall interrupt
	}
    if (Accelerometer.getDATA_FORMAT()!=0x0a){
    	Accelerometer.setDATA_FORMAT(0x0a);  //Full resolution +- 8g
    }
    if (Accelerometer.getPOWER_CTL()!=0x08){
    	Accelerometer.setPOWER_CTL(0x08); //Always measure and never sleep
    }
    if (Accelerometer.getBW_RATE()!=0x0a){
    	Accelerometer.setBW_RATE(0x0a);  //0x0a fo 100Hz, 0x0c for 400hz
    }

    /* Gyroscope Initialization */
    L3G4200DGyroscope Gyroscope(6,0x69); //0x69 is the Gyroscope address

    if (Gyroscope.getCTRL_REG1()!=0x0f){
    	Gyroscope.setCTRL_REG1(0x0f); //Enables three axis and sets 100hz
    }
    if (Gyroscope.getCTRL_REG2()!=0x20){
    	Gyroscope.setCTRL_REG2(0x20); //Normal mode no high Pass
    }
    if (Gyroscope.getCTRL_REG3()!=0x00){
       	Gyroscope.setCTRL_REG3(0x00); //FIFO and interrupt configuration
    }
    if (Gyroscope.getCTRL_REG4()!=0x00){
       	Gyroscope.setCTRL_REG4(0x00); //Data format and bus selection
    }
    if (Gyroscope.getCTRL_REG5()!=0x00){
       	Gyroscope.setCTRL_REG5(0x00); //FIFO and interrupt enable
    }

    cout << "And we are out" << endl;

    /* Gyro test */
    while(1){
    	Gyroscope.readFullSensorState();
    	int x = Gyroscope.getAngularRateX();
    	int y = Gyroscope.getAngularRateY();
    	int z = Gyroscope.getAngularRateZ();
    	cout << x << "," << y << "," << z << " --- ";
    	//sleep(0.1);
    //}

    /* ACC test */
    //while(1){
		Accelerometer.readFullSensorState();
		int xx = Accelerometer.getAccelerationX();
		int yy = Accelerometer.getAccelerationY();
		int zz = Accelerometer.getAccelerationZ();
		cout << xx << "," << yy << "," << zz << endl;
		sleep(0.1);
	}

	cout << "Out of program" << endl;

	return 0;
}
