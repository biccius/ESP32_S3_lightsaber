/*
 * RGBW.cpp
 *
 *  Created on: 12 apr 2024
 *      Author: Utente
 */

#include "RGBW.h"

//https://github.com/fellipecouto/PWMOutESP32/releases



/*

#if 0
void lama_normale()
{

	ledDriver.WRITE_PWM(PWM_RED_PIN, 0);
	ledDriver.WRITE_PWM(PWM_BLUE_PIN, 1024);
	ledDriver.WRITE_PWM(PWM_GREEN_PIN, 256);
	ledDriver.WRITE_PWM(PWM_WHITE_PIN, 1024);

}

void lama_verde()
{
	ledDriver.WRITE_PWM(PWM_RED_PIN, 0);
	ledDriver.WRITE_PWM(PWM_BLUE_PIN, 0);
	ledDriver.WRITE_PWM(PWM_GREEN_PIN, 1024);
	ledDriver.WRITE_PWM(PWM_WHITE_PIN, 1024);
}

void lama_rossa()
{
	ledDriver.WRITE_PWM(PWM_RED_PIN, 1024);
	ledDriver.WRITE_PWM(PWM_BLUE_PIN, 0);
	ledDriver.WRITE_PWM(PWM_GREEN_PIN, 0);
	ledDriver.WRITE_PWM(PWM_WHITE_PIN, 0);
}
#endif



 */


RGBW::RGBW(uint8_t _PWM_RED_PIN, uint8_t _PWM_GREEN_PIN, uint8_t _PWM_BLUE_PIN, uint8_t _PWM_WHITE_PIN)
{

//E (372) ledc: ledc_get_duty(740): LEDC is not initialized

	// TODO Auto-generated constructor stub
	pinMode(_PWM_RED_PIN	,	OUTPUT);
	pinMode(_PWM_GREEN_PIN	,	OUTPUT);
	pinMode(_PWM_BLUE_PIN	,	OUTPUT);
	pinMode(_PWM_WHITE_PIN	,	OUTPUT);

	digitalWrite (_PWM_RED_PIN		, LOW);
	digitalWrite (_PWM_GREEN_PIN	, LOW);
	digitalWrite (_PWM_BLUE_PIN		, LOW);
	digitalWrite (_PWM_WHITE_PIN	, LOW);

	//PWM = PWMOutESP32(PWM_BIT_RESOLUTION, PWM_FREQUENCY);
	ledcSetup(PWM_CHANNEL_RED, PWM_FREQUENCY_RED, PWM_BIT_RESOLUTION);
	ledcSetup(PWM_CHANNEL_GREEN, PWM_FREQUENCY_GREEN, PWM_BIT_RESOLUTION);
	ledcSetup(PWM_CHANNEL_BLUE, PWM_FREQUENCY_BLUE, PWM_BIT_RESOLUTION);
	ledcSetup(PWM_CHANNEL_WHITE, PWM_FREQUENCY_WHITE, PWM_BIT_RESOLUTION);


	_redPin = _PWM_RED_PIN;
	_greenPin = _PWM_GREEN_PIN;
	_bluePin = _PWM_BLUE_PIN;
	_whitePin = _PWM_WHITE_PIN;

	ledcAttachPin(_PWM_RED_PIN, PWM_CHANNEL_RED);
	ledcAttachPin(_PWM_GREEN_PIN, PWM_CHANNEL_GREEN);
	ledcAttachPin(_PWM_BLUE_PIN, PWM_CHANNEL_BLUE);
	ledcAttachPin(_PWM_WHITE_PIN, PWM_CHANNEL_WHITE);

}

void RGBW::WRITE_PWM(uint8_t PWM_CHANNEL_NUMBER, int PWM_VALUE )
{
	//ledcWrite(PWM_CHANNEL_NUMBER, dutyCycle);
}

RGBW::~RGBW() {
	// TODO Auto-generated destructor stub
}

