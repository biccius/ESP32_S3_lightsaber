#include <stdbool.h>


class MyMPU6050
{

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

	void normalizeGyro(float gyro_x, float gyro_y, float gyro_z);
	void normalizeAccel(float accelX,float accelY, float accelZ);

	void smoothSwing(float alpha, float max_step);

	float exponentialSmoothing(float current_value, float previous_value, float alpha);
	float limitVariation(float current_value, float previous_value, float max_step);

private:

	long accelXPresent, accelYPresent, accelZPresent;
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

}

