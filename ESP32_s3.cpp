/*
 * pin 1 - D2                |  Micro SD card     |
 * pin 2 - D3                |                   /
 * pin 3 - CMD               |                  |__
 * pin 4 - VDD (3.3V)        |                    |
 * pin 5 - CLK               | 8 7 6 5 4 3 2 1   /
 * pin 6 - VSS (GND)         | ▄ ▄ ▄ ▄ ▄ ▄ ▄ ▄  /
 * pin 7 - D0                | ▀ ▀ █ ▀ █ ▀ ▀ ▀ |
 * pin 8 - D1                |_________________|
 *                             ║ ║ ║ ║ ║ ║ ║ ║
 *                     ╔═══════╝ ║ ║ ║ ║ ║ ║ ╚═════════╗
 *                     ║         ║ ║ ║ ║ ║ ╚══════╗    ║
 *                     ║   ╔═════╝ ║ ║ ║ ╚═════╗  ║    ║
 * Connections for     ║   ║   ╔═══╩═║═║═══╗   ║  ║    ║
 * full-sized          ║   ║   ║   ╔═╝ ║   ║   ║  ║    ║
 * SD card             ║   ║   ║   ║   ║   ║   ║  ║    ║

 * ESP32-S3-USB-OTG | 38  37  GND  36 3V3 GND  35 34  33  |
 * Pin name         | D1  D0  VSS CLK VDD VSS CMD D3  D2  |
 * SD pin number    |  8   7   6   5   4   3   2   1   9 /
 *                  |                                  █/
 *                  |__▍___▊___█___█___█___█___█___█___/

 *    For more info see file README.md in this library or on URL:
 *    https://github.com/espressif/arduino-esp32/tree/master/libraries/SD_MMC
 */


#include "ESP32_S3.h"

#define USE_DMP 				0 		// MPU6050 DMP
#define USE_SMOOTHSWING_V1  	1
#define USE_SMOOTHSWING_V1S  	0
#define USE_SMOOTHSWING_V2		0
#define USE_SMOOTHSWING_V2S		0
#define  USE_SMOOTHSWING_TEST   0


//RGBW ledDriver(PWM_RED_PIN, PWM_GREEN_PIN, PWM_BLUE_PIN, PWM_WHITE_PIN);


MPU6050 accelgyro(0x68);	// A0 pin to GND

int16_t ax, ay, az;
int16_t gx, gy, gz;

AverageValue<int16_t> averageValueaX(100);
AverageValue<int16_t> averageValueaY(100);
AverageValue<int16_t> averageValueaZ(100);

AverageValue<int16_t> averageValuegX(100);
AverageValue<int16_t> averageValuegY(100);
AverageValue<int16_t> averageValuegZ(100);



unsigned long elapsed = 0;

unsigned long lastTime;


MOTION myMotion = MOTION();

#if USE_DMP

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gg;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
#endif
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------




void setup()
{

	Serial.begin(115200);

#if 0
    ledDriver.WRITE_PWM(PWM_RED_PIN, 0);
    ledDriver.WRITE_PWM(PWM_BLUE_PIN, 0);
    ledDriver.WRITE_PWM(PWM_GREEN_PIN, 0);
    ledDriver.WRITE_PWM(PWM_WHITE_PIN, 0);
#endif

    pinMode (PIN_BUTTON_A, INPUT_PULLUP);
    pinMode (PIN_BUTTON_B, INPUT_PULLUP);
    pinMode	(ANALOG_PIN_POT, INPUT);

    Serial.println("START");

    Wire.begin(I2C_SDA_MPU6050,I2C_SCL_MPU6050,400000UL);

    Serial.println("I2C OK");
#if USE_DMP
	pinMode(INTERRUPT_MPU6050, INPUT);
#endif


#if 1
    i2s_driver_install(i2s_num, &i2s_config, 0, NULL); // i2s.h
    i2s_set_pin(i2s_num, &pin_config);				   // i2s.h

    Serial.println("I2S OK");

    Wav1.Repeat=true;                                  // HUM
    Wav1.Playing=true;
    Wav2.Repeat=true;                                 	// SWING A
    Wav2.Playing=true;
    Wav3.Repeat=true;                                   // SWING B
    Wav3.Playing=true;


	if(! SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_D0, SDMMC_D1, SDMMC_D2, SDMMC_D3))
    {
        Serial.println("Pin change failed!");
        return;
    }

    if(!SD_MMC.begin()){
        Serial.println("Card Mount Failed");
        return;
    }

    Serial.println("SD CARD OK");
#endif


#if 1
    accelgyro.initialize();
#if USE_DMP
    accelgyro.dmpInitialize();
#endif

    Serial.println(accelgyro.testConnection() ? "MPU6050 OK" : "MPU6050 FAILED");
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	// 0x0
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);	// 0x0

     /*
     * FS_SEL | Full Scale Range   | LSB Sensitivity
     * -------+--------------------+----------------
     * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
     * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
     * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
     * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
     *
     *
     *16
     *  AFS_SEL | Full Scale Range | LSB Sensitivity
	 * --------+------------------+----------------
	 * 0       | +/- 2g           | 16384 LSB/mg
	 * 1       | +/- 4g           | 8192 LSB/mg
	 * 2       | +/- 8g           | 4096 LSB/mg
	 * 3       | +/- 16g          | 2048 LSB/mg
     */

#endif

     InitWavFiles();

#if 1
	     Serial.println("CalibrateAccel: ");
	     accelgyro.CalibrateAccel(25);
	     Serial.println("CalibrateGyro: ");
	     accelgyro.CalibrateGyro(25);

	     Serial.println("Offset rilevat:");
	     accelgyro.PrintActiveOffsets();

	     accelgyro.setXAccelOffset(accelgyro.getXAccelOffset());
	     accelgyro.setYAccelOffset(accelgyro.getYAccelOffset());
	     accelgyro.setZAccelOffset(accelgyro.getZAccelOffset());
	     accelgyro.setXGyroOffset(accelgyro.getXGyroOffset());
	     accelgyro.setYGyroOffset(accelgyro.getYGyroOffset());
	     accelgyro.setZGyroOffset(accelgyro.getZGyroOffset());

#endif

#if USE_DMP


	   accelgyro.setDMPEnabled(true);

	   attachInterrupt(digitalPinToInterrupt(INTERRUPT_MPU6050), dmpDataReady, RISING);

	   mpuIntStatus = accelgyro.getIntStatus();

	   dmpReady = true;

	   packetSize = accelgyro.dmpGetFIFOPacketSize();
#endif


}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop()
{


	PlayWavs();

#if USE_DMP

	if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer))
	{

		accelgyro.dmpGetAccel			(&aa, 		fifoBuffer	);
		accelgyro.dmpGetGyro			(&gg, 		fifoBuffer	);
		accelgyro.dmpGetQuaternion		(&q, 		fifoBuffer	);
		accelgyro.dmpGetGravity			(&gravity, 	&q			);
		accelgyro.dmpGetYawPitchRoll	(ypr, 		&q, &gravity);
#else

		accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
#endif

		averageValueaX.push(ax);	//
		averageValueaY.push(ay);	//
		averageValueaZ.push(az);	//

		averageValuegX.push(gx/131.0);	// 	(°/s)
		averageValuegY.push(gy/131.0);	// 	(°/s)
		averageValuegZ.push(gz/131.0);	// 	(°/s)



#if USE_SMOOTHSWING_V1
		myMotion.SmoothSwingV1(micros(),
				averageValuegX.average(),
				averageValuegY.average(),
				averageValuegZ.average(),
				true);

		myMotion.debug_SmoothSwing();
#endif
#if USE_SMOOTHSWING_V1S
		myMotion.SmoothSwingV1S(micros(),
				averageValuegX.average(),
				averageValuegY.average(),
				averageValuegZ.average(),
				true);

		myMotion.debug_SmoothSwing();
#endif
#if USE_SMOOTHSWING_V2
		myMotion.SmoothSwingV2(micros(),
				averageValuegX.average(),
				averageValuegY.average(),
				averageValuegZ.average(),
				true);

		myMotion.debug_SmoothSwing();
#endif
#if USE_SMOOTHSWING_V2S
		myMotion.SmoothSwingV2S(micros(),
				averageValuegX.average(),
				averageValuegY.average(),
				averageValuegZ.average(),
				true);
		myMotion.debug_SmoothSwing();

#endif
#if USE_SMOOTHSWING_TEST
		myMotion.Test_SmoothSwing(
				averageValueaX.average(),
				averageValueaY.average(),
				averageValueaZ.average(),
				averageValuegX.average(),
				averageValuegY.average(),
				averageValuegZ.average());

		myMotion.debug_my_smoothswing();

#endif

		 VolumeSwingH = myMotion.SmoothSwing_GetSwingAGain();
		 VolumeSwingL = myMotion.SmoothSwing_GetSwingBGain();
		 VolumeHum = myMotion.SmoothSwing_GetHumGain();





#if 0
		if (myMotion.DetectImpact(millis(), 200, averageValueaX.average(), averageValueaZ.average()))
		{
			Serial.print(millis());
			Serial.println("\tCLASH");

		}
		if (myMotion.DetectSwing(millis(), 200, averageValuegX.average(), averageValuegY.average(),averageValuegZ.average()))
		{
			Serial.print(millis());
			Serial.println("\tSWING");
		}
		if (myMotion.DetectSwingDirectionChange(averageValuegX.average(), averageValuegY.average(),averageValuegZ.average()))
		{
			Serial.print(millis());
			Serial.println("\tDIRECTION CHANGE");
		}

#endif


		 //VolumeSwingH = mapf(analogRead(ANALOG_PIN_POT),0.0,4095.0,0.0,1.0);
		//VolumeSwingH = myMotion.SmoothSwing_GetSwingA();
		// VolumeSwingL = myMotion.SmoothSwing_GetSwingB();
		// VolumeHum = myMotion.SmoothSwing_GetHumGain();

#if 0
		 if (millis() - elapsed > 100)
		 {
			 //plot(VolumeSwingH);

			 //Serial.print(averageValuegX.average()); Serial.print("\t");
			 //Serial.print(averageValuegY.average()); Serial.print("\t");
			 //Serial.print(averageValuegZ.average());Serial.print("\t");

			 Serial.print(VolumeSwingH); Serial.print("\t");
			// Serial.print(VolumeSwingL); Serial.print("\t");
			// Serial.print(VolumeHum); Serial.print("\t");
			 //Serial.print(myMotion.SmoothSwing_GetSwingStrength());  Serial.print("\t");

			 //Serial.print(digitalRead(PIN_BUTTON_A)); Serial.print("\t");
			// Serial.print(digitalRead(PIN_BUTTON_A)); Serial.print("\t");
			// Serial.print(VolumeSwingH); Serial.print("\t");

			 Serial.println();


			 elapsed = millis();

		 }
#endif
#if USE_DMP
	}
#endif
}

