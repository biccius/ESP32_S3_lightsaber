/*
 * MyMPU6050.cpp
 *
 *  Created on: 1 lug 2024
 *      Author: Utente
 */

#include "MyMPU6050.h"
#include <Wire.h>
#include "Arduino.h"
#include <stdint.h>




#define G_FORCE 9.80665


MyMPU6050::MyMPU6050()
{
	// TODO Auto-generated constructor stub

}

MyMPU6050::~MyMPU6050() {
	// TODO Auto-generated destructor stub
}

void MyMPU6050::init_MyMPU6050(bool enableInterrupt)
{
	// power management
	Wire.beginTransmission(0x68);          // Start the communication by using address of MPU
	Wire.write(0x6B);                           // Access the power management register
	Wire.write(0b00000000);                     // Set sleep = 0
	Wire.endTransmission();                     // End the communication

	// configure gyro
	Wire.beginTransmission(0x68);
	Wire.write(0x1B);                           // Access the gyro configuration register
	Wire.write(0b00000000);						// GYRO_FULL_SCALE_250_DPS

	/*#define GYRO_FULL_SCALE_250_DPS  0x00
	  #define GYRO_FULL_SCALE_500_DPS  0x08
	  #define GYRO_FULL_SCALE_1000_DPS 0x10
	  #define GYRO_FULL_SCALE_2000_DPS 0x18
	*/

	Wire.endTransmission();

	// configure accelerometer
	Wire.beginTransmission(0x68);
	Wire.write(0x1C);                           // Access the accelerometer configuration register
	Wire.write(0b00000000);						// ACC_FULL_SCALE_2G

	/*
		#define ACC_FULL_SCALE_2G  0x00
		#define ACC_FULL_SCALE_4G  0x08
		#define ACC_FULL_SCALE_8G  0x10
		#define ACC_FULL_SCALE_16G 0x18
	*/

	if (enableInterrupt)
	{
		Wire.beginTransmission(0x68);          		// Start the communication by using address of MPU
		Wire.write(0x38);                           // Interrupt regisers
		Wire.write(0b00000001);                     // Enable interrupt pin for raw data
		Wire.endTransmission();                     // End the communication

	}


}


void MyMPU6050::getGyroValues()
{
  Wire.beginTransmission(0b1101000);                          // Start the communication by using address of MPU
  Wire.write(0x43);                                           // Access the starting register of gyro readings
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);                              	  // Request for 6 bytes from gyro registers (43 - 48)
  while(Wire.available() < 6);                                // Wait untill all 6 bytes are available
  gyroXPresent = Wire.read()<<8|Wire.read();                  // Store first two bytes into gyroXPresent
  gyroYPresent = Wire.read()<<8|Wire.read();                  // Store next two bytes into gyroYPresent
  gyroZPresent = Wire.read()<<8|Wire.read();                  //Store last two bytes into gyroZPresent
}

void MyMPU6050::calibrateGyroValues(int Count)
{

	Serial.println("Calibration...");
    for (int i=0; i<Count; i++)
    {
      getGyroValues();
      gyroXCalli = gyroXCalli + gyroXPresent;
      gyroYCalli = gyroYCalli + gyroYPresent;
      gyroZCalli = gyroZCalli + gyroZPresent;
    }
    gyroXCalli = gyroXCalli/Count;
    gyroYCalli = gyroYCalli/Count;
    gyroZCalli = gyroZCalli/Count;

	Serial.println("Completed!");
}


void MyMPU6050::getAngularVelocity()
{
  rotX = gyroXPresent / 131.0;
  rotY = gyroYPresent / 131.0;
  rotZ = gyroZPresent / 131.0;
}


void MyMPU6050::readAndProcessGyroData()
{
  gyroXPast = gyroXPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroYPast = gyroYPresent;                                   // Assign Present gyro reaging to past gyro reading
  gyroZPast = gyroZPresent;                                   // Assign Present gyro reaging to past gyro reading
  timePast = timePresent;                                     // Assign Present time to past time
  timePresent = millis();                                     // get the current time in milli seconds, it is the present time

  getGyroValues();                                            // get gyro readings
  getAngularVelocity();                                       // get angular velocity
  calculateAngle();                                           // calculate the angle

}


void MyMPU6050::calculateAngle()
{
  // same equation can be written as
  // angelZ = angelZ + ((timePresentZ - timePastZ)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) / (2*1000*131);
  // 1/(1000*2*131) = 0.00000382
  // 1000 --> convert milli seconds into seconds
  // 2 --> comes when calculation area of trapezium
  // substacted the callibated result two times because there are two gyro readings
  angelX = angelX + ((timePresent - timePast)*(gyroXPresent + gyroXPast - 2*gyroXCalli)) * 0.00000382;
  angelY = angelY + ((timePresent - timePast)*(gyroYPresent + gyroYPast - 2*gyroYCalli)) * 0.00000382;
  angelZ = angelZ + ((timePresent - timePast)*(gyroZPresent + gyroZPast - 2*gyroZCalli)) * 0.00000382;
}

void MyMPU6050::printData()
{
	  Serial.println("Gyro (deg/sec)");
	  Serial.print(" X=");
	  Serial.print(rotX);
	  Serial.print(" Y=");
	  Serial.println(rotY);
	  Serial.print("Angle (deg)");
	  Serial.print(" Z=");
	  Serial.println(rotZ);

	  Serial.println("Angular displacement wrt started position (deg)");
	  Serial.print("angel of X axis=");
	  Serial.print(angelX);
	  Serial.print(" angel of Y axis=");
	  Serial.print(angelY);
	  Serial.print(" angel of Z axis=");
	  Serial.println(angelZ);

	  Serial.println("Acceleration (g)");
	  Serial.print(" X=");
	  Serial.print(gForceX);
	  Serial.print(" Y=");
	  Serial.print(gForceY);
	  Serial.print(" Z=");
	  Serial.println(gForceZ);
}

// -------------------------------------------------

void MyMPU6050::normalize_gyro()
{
  // Sensitivity Scale Factor (MPU datasheet page 8)
	normalGyroX= gyroXPresent / 32.8;
	normalGyroY = gyroYPresent / 32.8;
	normalGyroZ = gyroZPresent / 32.8;
}

void MyMPU6050::normalize_accel()
{
  // Sensitivity Scale Factor (MPU datasheet page 9)
	normalAccelX = accelX * G_FORCE / 16384;
	normalAccelY= accelY * G_FORCE / 16384;
	normalAccelZ = accelZ * G_FORCE / 16384;
}


void MyMPU6050::readAndProcessAccelData() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();
  accelY = Wire.read()<<8|Wire.read();
  accelZ = Wire.read()<<8|Wire.read();
  processAccelData();
}


void MyMPU6050::processAccelData()
{
	  gForceX = accelX/16384.0;
	  gForceY = accelY/16384.0;
	  gForceZ = accelZ/16384.0;
}
