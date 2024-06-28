/*
 * RGBW.h
 *
 *  Created on: 12 apr 2024
 *      Author: Utente
 */

#ifndef RGBW_H_
#define RGBW_H_

#include "Arduino.h"


#define 	PWM_BIT_RESOLUTION 		10

#define		PWM_CHANNEL_RED 		0x00
#define		PWM_CHANNEL_GREEN 		0x01
#define		PWM_CHANNEL_BLUE 		0x02
#define		PWM_CHANNEL_WHITE 		0x03


#define		PWM_FREQUENCY_RED 		1000
#define		PWM_FREQUENCY_GREEN 	1000
#define		PWM_FREQUENCY_BLUE 		1000
#define		PWM_FREQUENCY_WHITE 	1000


class RGBW
{
	public:

		RGBW( uint8_t _PWM_RED_PIN,	uint8_t _PWM_GREEN_PIN,	uint8_t _PWM_BLUE_PIN,	uint8_t _PWM_WHITE_PIN);

		virtual ~RGBW();

		void WRITE_PWM(uint8_t PWM_CHANNEL_NUMBER, int PWM_VALUE );



	private:

		uint8_t _redPin;
		uint8_t _greenPin;
		uint8_t _bluePin;
		uint8_t _whitePin;





};

#endif /* RGBW_H_ */
