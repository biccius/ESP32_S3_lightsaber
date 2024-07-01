/*
 * MyMPU6050.h
 *
 *  Created on: 1 lug 2024
 *      Author: Utente
 */

#ifndef MYMPU6050_H_
#define MYMPU6050_H_

#include <stdbool.h>


class MyMPU6050 {
public:
	MyMPU6050();
	virtual ~MyMPU6050();

	void init_MyMPU6050(bool enableInterrupt);
	void calibrateGyroValues(int Count);
	void getGyroValues();
	void readAndProcessGyroData();
	void readAndProcessAccelData();
	void getAngularVelocity();
	void calculateAngle();
	void printData();
	void processAccelData();



private:

	long accelX, accelY, accelZ;
	float gForceX, gForceY, gForceZ;
	long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
	long gyroXPresent = 0, gyroYPresent = 0, gyroZPresent = 0;
	float normalGyroX,normalGyroY,normalGyroZ;
	float normalAccelX, normalAccelY, normalAccelZ;

	long gyroXPast = 0, gyroYPast = 0, gyroZPast = 0;
	float rotX, rotY, rotZ;
	float angelX = 0, angelY = 0, angelZ = 0;
	long timePast = 0;
	long timePresent = 0;

	float normalTemperature;






#endif /* MYMPU6050_H_ */
