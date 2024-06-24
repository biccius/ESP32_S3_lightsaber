// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _ESP32_s3_H_
#define _ESP32_s3_H_
//add your includes for the project ESP32_s3 here

#include <Arduino.h>
#include "pin_definition.h"
#include "driver/i2s.h"                 //i2s legacy routines
#include "i2s_config.h"
#include "Wire.h"
#include "WAV.h"
#include "FS.h"
#include "SD_MMC.h"
#include "RGBW.h"
//#include "MPU6050.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <AverageValue.h>
#include "MOTION.h"
#include <PlotPlus.h>


// the following definition can replace the default output stream (Serial)
#define PLOT_PLUS_STREAM Serial

//the following completely disables the plot statements
#define PLOT_PLUS true

//end of add your includes here


static const i2s_port_t i2s_num = I2S_NUM_0;  // i2s port number

//add your function definitions for the project ESP32_s3 here



//Do not add code below this line
#endif /* _ESP32_s3_H_ */
