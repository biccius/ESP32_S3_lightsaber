/*
 * pin_definition.h
 *
 *  Created on: 3 apr 2024
 *      Author: fabry
 */

#ifndef PIN_DEFINITION_H_
#define PIN_DEFINITION_H_

		  // I2S MAX9875
          #define I2S_DOUT      2          // verde:i2S Data out oin
          #define I2S_BCLK      1          // giallo: Bit clock
          #define I2S_LRC       4          // blu: Left/Right clock, also known as Frame clock or word select

		  // SD MMC 4bit
   		  #define SDMMC_CLK  	36
		  #define SDMMC_CMD  	35
		  #define SDMMC_D0   	37
		  #define SDMMC_D1   	38
		  #define SDMMC_D2   	33
		  #define SDMMC_D3   	34 // GPIO 39 is not broken-out on ESP32-S3-DevKitC-1 v1.1

		  // CREE LED DRIVER
		  #define PWM_RED_PIN	5
		  #define PWM_GREEN_PIN	7
		  #define PWM_BLUE_PIN	12
		  #define PWM_WHITE_PIN	6

		  // I2C MPU6050
		  #define I2C_SDA_MPU6050 8
		  #define I2C_SCL_MPU6050 9
		  #define INTERRUPT_MPU6050 15

		#define PIN_BUTTON_A 	13
		#define PIN_BUTTON_B 	14
		#define ANALOG_PIN_POT 	17

#endif /* PIN_DEFINITION_H_ */
