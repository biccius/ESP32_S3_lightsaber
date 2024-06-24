/*
 * MOTION.h
 *
 *  Created on: 19 giu 2024
 *      Author: Utente
 */

#ifndef MOTION_H_
#define MOTION_H_

#include "stdint.h"

class MOTION {
public:
	MOTION();
	virtual ~MOTION();

	bool DetectSwingDirectionChange(float GX,float GY,float GZ);
	bool DetectSwing(uint32_t currentTime, uint32_t supressTime,float GX, float GY, float GZ);
	bool DetectImpact(uint32_t currentTime, uint32_t supressTime, float AX, float AZ);

	void SmoothSwingV1(uint32_t nCurrentTime, float gx, float gy, float gz, bool useMicros);
	void SmoothSwingV1S(uint32_t nCurrentTime, float gx, float gy, float gz, bool useMicros);

	void SmoothSwingV2(uint32_t nCurrentTime, float gx, float gy, float gz,bool useMicros);
	void SmoothSwingV2S(uint32_t nCurrentTime, float gx, float gy, float gz, bool useMicros);
	float SmoothSwing_GetSwingStrength();
	float SmoothSwing_GetSwingA();
	float SmoothSwing_GetSwingB();
	float SmoothSwing_GetSwingAGain();
	float SmoothSwing_GetSwingBGain();
	float SmoothSwing_GetHumGain();
	void debug_SmoothSwing();

	void Test_SmoothSwing(float ax, float ay, float az, float gx, float gy, float gz);
	void debug_my_smoothswing();

};

#endif /* MOTION_H_ */
