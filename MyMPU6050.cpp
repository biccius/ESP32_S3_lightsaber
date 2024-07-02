#include "MyMPU6050.h"
#include <Wire.h>
#include "Arduino.h"
#include <stdint.h>

#define G_FORCE 9.80665

MyMPU6050::MyMPU6050()
{
}

MyMPU6050::~MyMPU6050()
{
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
    Wire.write(0b00000000);                     // GYRO_FULL_SCALE_250_DPS
    Wire.endTransmission();

    // configure accelerometer
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);                           // Access the accelerometer configuration register
    Wire.write(0b00000000);                     // ACC_FULL_SCALE_2G
    Wire.endTransmission();

    if (enableInterrupt)
    {
        Wire.beginTransmission(0x68);           // Start the communication by using address of MPU
        Wire.write(0x38);                       // Interrupt registers
        Wire.write(0b00000001);                 // Enable interrupt pin for raw data
        Wire.endTransmission();                 // End the communication
    }
}

void MyMPU6050::getGyroValues()
{
    Wire.beginTransmission(0x68);           // Start the communication by using address of MPU
    Wire.write(0x43);                       // Access the starting register of gyro readings
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);              // Request for 6 bytes from gyro registers (43 - 48)
    while (Wire.available() < 6);           // Wait until all 6 bytes are available
    gyroXPresent = Wire.read() << 8 | Wire.read();  // Store first two bytes into gyroXPresent
    gyroYPresent = Wire.read() << 8 | Wire.read();  // Store next two bytes into gyroYPresent
    gyroZPresent = Wire.read() << 8 | Wire.read();  // Store last two bytes into gyroZPresent
}

void MyMPU6050::calibrateGyroValues(int Count)
{
    Serial.println("Calibration...");
    for (int i = 0; i < Count; i++)
    {
        getGyroValues();
        gyroXCalli += gyroXPresent;
        gyroYCalli += gyroYPresent;
        gyroZCalli += gyroZPresent;
    }
    gyroXCalli /= Count;
    gyroYCalli /= Count;
    gyroZCalli /= Count;

    Serial.println("Completed!");
}

void MyMPU6050::getAngularVelocity()
{
    rotX = gyroXPresent / 131.0;
    rotY = gyroYPresent / 131.0;
    rotZ = gyroZPresent / 131.0;
}

void MyMPU6050::calculateAngle()
{
    float deltaTime = (timePresent - timePast) * 0.00000382;
    angelX += (gyroXPresent + gyroXPast - 2 * gyroXCalli) * deltaTime;
    angelY += (gyroYPresent + gyroYPast - 2 * gyroYCalli) * deltaTime;
    angelZ += (gyroZPresent + gyroZPast - 2 * gyroZCalli) * deltaTime;
}

void MyMPU6050::readAndProcessGyroData()
{
    gyroXPast = gyroXPresent;
    gyroYPast = gyroYPresent;
    gyroZPast = gyroZPresent;
    timePast = timePresent;
    timePresent = millis();

    getGyroValues();
    getAngularVelocity();
    calculateAngle();
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
    Serial.print("Angle of X axis=");
    Serial.print(angelX);
    Serial.print(" Angle of Y axis=");
    Serial.print(angelY);
    Serial.print(" Angle of Z axis=");
    Serial.println(angelZ);

    Serial.println("Acceleration (g)");
    Serial.print(" X=");
    Serial.print(gForceX);
    Serial.print(" Y=");
    Serial.print(gForceY);
    Serial.print(" Z=");
    Serial.println(gForceZ);
}

void MyMPU6050::normalizeGyro(float gyro_x, float gyro_y, float gyro_z)
{

    normalGyroX = gyro_x / 32.8;
    normalGyroY = gyro_y / 32.8;
    normalGyroZ = gyro_z / 32.8;
}

void MyMPU6050::normalizeAccel(float accelX,float accelY, float accelZ)
{
    // Sensitivity Scale Factor (MPU datasheet page 9)
    normalAccelX = accelX * G_FORCE / 16384;
    normalAccelY = accelY * G_FORCE / 16384;
    normalAccelZ = accelZ * G_FORCE / 16384;
}

void MyMPU6050::readAndProcessAccelData()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    while (Wire.available() < 6);
    accelXPresent = Wire.read() << 8 | Wire.read();
    accelYPresent = Wire.read() << 8 | Wire.read();
    accelZPresent = Wire.read() << 8 | Wire.read();
    processAccelData();
}

void MyMPU6050::processAccelData()
{
    gForceX = accelXPresent / 16384.0;
    gForceY = accelYPresent / 16384.0;
    gForceZ = accelZPresent / 16384.0;
}




float MyMPU6050::limitVariation(float current_value, float previous_value, float max_step)
{
    float delta = current_value - previous_value;
    if (abs(delta) > max_step)
    {
        if (delta > 0)
        {
            current_value = previous_value + max_step;
        }
        else
        {
            current_value = previous_value - max_step;
        }
    }
    return current_value;
}



void MyMPU6050::smoothSwing(float alpha, float max_step)
{
    // Valori filtrati statici
    static float filtered_hum = 0.0;
    static float filtered_swing_a = 0.0;
    static float filtered_swing_b = 0.0;

    // Converti i valori grezzi del giroscopio in float
       float accel_x = static_cast<float>(accelXPresent);
       float accel_y = static_cast<float>(accelYPresent);
       float accel_z = static_cast<float>(accelZPresent);


    // Converti i valori grezzi del giroscopio in float
    float gyro_x = static_cast<float>(gyroXPresent);
    float gyro_y = static_cast<float>(gyroYPresent);
    float gyro_z = static_cast<float>(gyroZPresent);

    // Normalizza i valori del giroscopio
    this->normalizeGyro(gyro_x,gyro_y,gyro_z);

    // Normalizza i valori dell'accellerazione
    this->normalizeAccel(accel_x,accel_y,accel_z);

    // Applicazione della media mobile esponenziale
    float smoothed_hum = this->exponentialSmoothing(gyro_x, filtered_hum, alpha);
    float smoothed_swing_a = this->exponentialSmoothing(gyro_y, filtered_swing_a, alpha);
    float smoothed_swing_b = this->exponentialSmoothing(gyro_z, filtered_swing_b, alpha);

    // Limitazione delle variazioni
    smoothed_hum = this->limitVariation(smoothed_hum, filtered_hum, max_step);
    smoothed_swing_a = this->limitVariation(smoothed_swing_a, filtered_swing_a, max_step);
    smoothed_swing_b = this->limitVariation(smoothed_swing_b, filtered_swing_b, max_step);

    // Aggiorna i valori filtrati
    filtered_hum = smoothed_hum;
    filtered_swing_a = smoothed_swing_a;
    filtered_swing_b = smoothed_swing_b;

    // Aggiorna i valori del giroscopio con i valori smussati
    gyroXPresent = static_cast<int16_t>(smoothed_hum);
    gyroYPresent = static_cast<int16_t>(smoothed_swing_a);
    gyroZPresent = static_cast<int16_t>(smoothed_swing_b);
}

float MyMPU6050::exponentialSmoothing(float current_value, float previous_value, float alpha)
{
    return alpha * current_value + (1.0 - alpha) * previous_value;
}
