/*
 * MOTION.cpp
 *
 *  Created on: 19 giu 2024
 *      Author: Utente
 */

#include "MOTION.h"
#include "math.h"
#include <stdlib.h>
#include "Arduino.h"
#include "stdint.h"

MOTION::MOTION() {
	// TODO Auto-generated constructor stub

}

MOTION::~MOTION() {
	// TODO Auto-generated destructor stub
}


static uint32_t LockOutDetectSwing;
static uint32_t LockOutDetectImpact;
static float ThresholdImpact = 7;
static float ThresholdSwingGyro = 600;

// algorithm for detection, can change to more complex algorithm later.
inline static bool GetDetectImpact(float AX,float AZ)
{
	if (abs(AX) + abs(AZ) + abs(AZ) > ThresholdImpact)
		return 1;
	else
		return 0;
}

inline static bool GetDetectSwing(float GX,float GY, float GZ)
{

	if ( (abs(GX) > 600 || abs(GY)  > 300 || abs(GZ)  > 300) )
		return 1;
	else
		return 0;
}


bool MOTION::DetectImpact(uint32_t currentTime, uint32_t supressTime, float AX, float AZ)
{
	if(currentTime < LockOutDetectImpact)
		return 0;

	if (GetDetectImpact(AX,AZ))
	{
		LockOutDetectSwing	 = currentTime + supressTime;
		LockOutDetectImpact	 = currentTime + supressTime;
		return 1;
	}
	else
		return 0;
}

bool MOTION::DetectSwing(uint32_t currentTime, uint32_t supressTime,float GX, float GY, float GZ)
{
	if(currentTime < LockOutDetectSwing)
		return 0;

	if (GetDetectSwing(GX,GY,GZ))
	{
		LockOutDetectSwing	 = currentTime + supressTime;
		return 1;
	}
	else
		return 0;
}

bool MOTION::DetectSwingDirectionChange(float GX,float GY,float GZ)
{
	static float lastGX;
		static float lastGY;
		static float lastGZ;

		static float lastGXAccel;
		static float lastGYAccel;
		static float lastGZAccel;

		float gxAccel = GX - lastGX;
		float gyAccel = GY - lastGY;
		float gzAccel = GZ - lastGZ;

		lastGX = GX;
		lastGY = GY;
		lastGZ = GZ;

		float magnitude = sqrtf(powf(gxAccel,2) + powf(gyAccel,2) + powf(gzAccel,2));

		gxAccel = GX / magnitude;
		gyAccel = GY / magnitude;
		gzAccel = GZ / magnitude;

		lastGXAccel = gxAccel;
		lastGYAccel = gyAccel;
		lastGZAccel = gzAccel;

		float dotProduct = gxAccel*lastGXAccel + gyAccel*lastGYAccel + gzAccel*lastGZAccel;

		if (dotProduct < 0) return true;
		else 				return false;
}



float SwingStrength;
float SmoothSwingSensitivity = 600;	// a total of 540 degrees per second or greater between all axes will apply the loudest swing sound
float SmoothSwingAmountA;
float SmoothSwingAmountB;

float m_flMultiHumCyclePeriod = 2*PI/1; // a full 2PI transition takes 1 seconds

// Saber is along x-axis

/******************************************************************************/
/*!
 * @name SmoothSwing V1
 * @brief
 */
/******************************************************************************/
float m_flMultiHumSensitivity = 540; 	// 540 degrees per second or greater will apply the max contribution from a paired hum

void MOTION::SmoothSwingV1(uint32_t nCurrentTime, float gx, float gy, float gz,bool useMicros)
{
	// gx, gy, gz are all instantaneous gyro values
	// flTotalRotation is the total angular velocity of gx, gy, gz
	// m_flMultiHumCyclePeriod is actually poorly named and is really the inverse of the period.

	float flTotalRotation = abs(gx) + abs(gy) + abs(gz);

	// Periodicity / Rotation of the basis
	float flCycleTime;
	if (useMicros) flCycleTime = ( nCurrentTime / 1000000.0f ) * m_flMultiHumCyclePeriod;
	else flCycleTime = ( nCurrentTime / 1000.0f ) * m_flMultiHumCyclePeriod;

	float flYAmount = sinf( flCycleTime );
	float flZAmount = cosf( flCycleTime );

	// Normalize to the swing sensitivity and clamp
	float flNormalizedY = max( -1.0f, min( 1.0f, gy / m_flMultiHumSensitivity ) );
	float flNormalizedZ = max( -1.0f, min( 1.0f, gz / m_flMultiHumSensitivity ) );

	// Apply the rotation
	float flMultiHumAmount0 = flYAmount * flNormalizedY;
	float flMultiHumAmount1 = flZAmount * flNormalizedZ;

	// Store off total strength
	SwingStrength = min( 1.0f, flTotalRotation / SmoothSwingSensitivity );

	// Multi-hum amount is the sum of both axes (both go from -1..1 range) divided by sqrt(2). Think of the case where both are 1.
	float m_flMultiHumAmount = ( flMultiHumAmount0 + flMultiHumAmount1 ) / 1.414214f;

	if (m_flMultiHumAmount > 0)
	{
		SmoothSwingAmountA 	= m_flMultiHumAmount;
		SmoothSwingAmountB 	= 0;
	}
	else
	{
		SmoothSwingAmountA 	= 0;
		SmoothSwingAmountB 	= abs(m_flMultiHumAmount);
	}
}

/******************************************************************************/
/*!
 * @name SmoothSwing V1S
 * @brief  A modified version of V1
 */
/******************************************************************************/
void SmoothSwingV1S(uint32_t nCurrentTime, float gx, float gy, float gz, bool useMicros)
{
	float flTotalRotation = sqrtf( powf(gx,2) + powf(gy,2) + powf(gz,2) );


	float flCycleTime;

	if (useMicros) flCycleTime = ( nCurrentTime / 1000000.0f ) * m_flMultiHumCyclePeriod;
	else  flCycleTime = ( nCurrentTime / 1000.0f ) * m_flMultiHumCyclePeriod;

	SwingStrength = min( 1.0f, flTotalRotation / SmoothSwingSensitivity );

	SmoothSwingAmountA 	= (sinf(flCycleTime			)+1)/2;
	SmoothSwingAmountB 	= (sinf(flCycleTime + PI	)+1)/2;
}

/******************************************************************************/
/*!
 * @name SmoothSwing V2
 * @brief
 */
/******************************************************************************/
//float g_flMaximumHumDucking = 0.75f;
float g_flSwingSharpness = 1.3; //<some value between 1.0 and 2.0>
float g_flSameSwingDirThreshold = 0.0f; // This seems to be a reasonable value, but experiment
float g_flSwingStrengthThreshold = 0.05f; // Again, configurable

float m_flMinPitchBoundaryRads = 10;
float m_flMaxPitchBoundaryRads = 60;

float m_flHalfTransition1AngleRads = 30; // changed to degrees
float m_flHalfTransition2AngleRads = 90;

float m_flPitchBoundary1;
float m_flPitchBoundary2;

float m_flSwingHiStart;
float m_flSwingLowEnd;

float m_flPitchBoundaryLerp;
float m_flSwingPitchLowHiLerp;

void MOTION::SmoothSwingV2(uint32_t nCurrentTime, float gx, float gy, float gz,bool useMicros)
{
	// read raw accel/gyro measurements from device
	// All values are in Gs for accel and degrees/s for gyro

	// Keep track of how long a "Frame" is.
	// Use this to integrate angular velocity in DoNewSwing

	static float m_nLastUpdateTime;

	float flTimeSinceLastUpdate;

	if (useMicros)	flTimeSinceLastUpdate = ( nCurrentTime - m_nLastUpdateTime ) / 1000000.0f;
	else flTimeSinceLastUpdate = ( nCurrentTime - m_nLastUpdateTime ) / 1000.0f;

	m_nLastUpdateTime = nCurrentTime;

	// Calculate magnitude of the angular velocity
	float flTotalAngularVelSq = powf(gx,2) + powf(gy,2) + powf(gz,2);
	float flTotalAngularVel = sqrtf( flTotalAngularVelSq );

	// Smooth this value out (box filter over X frames)
//	m_flSmoothedTotalAngularVel.StoreValue( flTotalAngularVel );
//	flTotalAngularVel = m_flSmoothedTotalAngularVel.GetSmoothedValue();

	// Calculate how HARD our swing is
	// This is a normalized value from 0..1, clamped at 1
	SwingStrength = min( 1.0f, flTotalAngularVel / SmoothSwingSensitivity );

	// instantaneous angular velocity vector
	// used for acceleration direction changes and smoothed by the called code
//	Vector vCurrentGyroValues;
//	vCurrentGyroValues.x = gx;
//	vCurrentGyroValues.y = gy;
//	vCurrentGyroValues.z = gz;

	// On second thought, when you've tried as many swing algorithms as I have, using the convention New, really doesn't mean anything.
	//DoNewSwing( vCurrentGyroValues, flTotalAngularVel , flTimeSinceLastUpdate );

	// How far have we travelled?
	static float m_flTotalRotationThisSwing;

	m_flTotalRotationThisSwing += flTotalAngularVel * flTimeSinceLastUpdate;

	// Wrap
	float flWrappedRotationThisSwing = fmod(m_flTotalRotationThisSwing, 2*PI);

	// See if we are in transition zone 1
	float flAngleToTransition1 =  m_flPitchBoundary1 - flWrappedRotationThisSwing;
	if ( flAngleToTransition1 > m_flPitchBoundary1 - m_flHalfTransition1AngleRads && flAngleToTransition1 < m_flPitchBoundary1 + m_flHalfTransition1AngleRads )
	{
		// How far across the zone are we?
		float flPercentAcross = ( flAngleToTransition1 + m_flHalfTransition1AngleRads ) / ( m_flHalfTransition1AngleRads * 2.0f );

		// That's how much we lerp between the two looping hum pairs
		m_flPitchBoundaryLerp = flPercentAcross;
	}

	// Pitch boundary 2 is only for spins. If you've gone at least 180 past the pitch boundary, assume it's a spin.
	// In the second pitch boundary, attenuate the swing strength and change the pitch lerp back to 0 so that the next time
	// through the pitch boundary produces yet another pitch shift. Attenuation hides the instant change in the pitch lerp.

	// See if we are in transition zone 2
	float flAngleToTransition2 = m_flPitchBoundary2 - flWrappedRotationThisSwing;
	if ( flAngleToTransition2 > m_flPitchBoundary2 - m_flHalfTransition2AngleRads && flAngleToTransition2 < m_flPitchBoundary2 + m_flHalfTransition2AngleRads )
	{
		// How far across the zone are we?
		float flPercentAcross = ( flAngleToTransition2 + m_flHalfTransition2AngleRads ) / ( m_flHalfTransition2AngleRads * 2.0f );

		// Since we're going back the other way, we do 1-flPercentAcross
		m_flPitchBoundaryLerp = 1.0f - flPercentAcross;
	}

	// Quick remap in case we flipped which sounds were start and end.
	// Note, the value of m_flPitchBoundaryLerp persists so that it stays as whatever it was left at in the event we leave a transition zone.
	m_flSwingPitchLowHiLerp = m_flSwingHiStart * ( 1.0f - m_flPitchBoundaryLerp ) + m_flSwingLowEnd * m_flPitchBoundaryLerp;

	// What's our current non-smoothed accel direction?
	// If we want to, we can assume that the average saber is about 1m in length.
	// Then we can think of the gyro values as the velocities at the tip of the saber.
	// The difference is a really bad approx for the first derivative.
	// Blah Blah Blah...
	// In reality, we're just looking for a change in acceleration big enough that we'll want to do a new sound
//	Vector vSwingAccel = vCurrentGyroValues - m_vLastGyroValues;
//	m_vLastGyroValues = vCurrentGyroValues;

	static float lastGX;
	static float lastGY;
	static float lastGZ;

	float gxAccel = gx - lastGX;
	float gyAccel = gy - lastGY;
	float gzAccel = gz - lastGZ;

	lastGX = gx;
	lastGY = gy;
	lastGZ = gz;

	float magnitude = sqrtf(powf(gxAccel,2) + pow(gyAccel,2) + powf(gzAccel,2));

	// We only care about the direction of the change, not the magnitude
	//vSwingAccel.NormalizeInPlace();
	gxAccel = gx / magnitude;
	gyAccel = gy / magnitude;
	gzAccel = gz / magnitude;

	// Box filter over the last 5 vSwingAccel values
//	Vector vSmoothedLastSwingAccel = GetSmoothedLastSwingAccel();

	// Determine if we've changed directions. We do this for cases where we may not actually go below our strength threshold, but have changed motion enough to warrant a new swing.
	// In order to change acceleration direction, we'll need to have approached a zero velocity at least along the projection of the dot product between the two accel vector changes.
	//float flSameDir = DotProduct( vSmoothedLastSwingAccel, vSwingAccel );

	static float lastGXAccel;
	static float lastGYAccel;
	static float lastGZAccel;

	float flSameDir = gxAccel*lastGXAccel + gyAccel*lastGYAccel + gzAccel*lastGZAccel;

	lastGXAccel = gxAccel;
	lastGYAccel = gyAccel;
	lastGZAccel = gzAccel;

	// Keep track of the value we calculated this time so that it is used to calculate the smoothed value for next time
	//AddLastSwingAccel( vSwingAccel );

	// Either of two conditions for new swing
	// 1. The "sameness" of the direction of our acceleration is below a certain threshold
	// 2. We're not moving much
	if ( ( flSameDir < g_flSameSwingDirThreshold ) || ( SwingStrength < g_flSwingStrengthThreshold ) )
	{
		// When the saber is still, this will be firing EVERY tick/frame
		// That's ok, because we shouldn't be playing any swings then, so rapidly picking new ones won't be noticed
		//OnNewSwing();

		// Select which exact pair to use
//		m_nHumPair = rand() % NUM_SWING_PAIRS;

		// Select swing high-low starts
		// Do we start with the high sound or the low sound from the pair?
		if ( rand() % 100 > 50 )
		{
			m_flSwingHiStart = 1.0f;
			m_flSwingLowEnd = 0.0f;
		}
		else
		{
			m_flSwingHiStart = 0.0f;
			m_flSwingLowEnd = 1.0f;
		}

		// Current lerp between the two sounds in the current pair
		m_flPitchBoundaryLerp = 0.0f;

		// How far have we rotated without changing directions?
		m_flTotalRotationThisSwing = 0.0f;

		// How far do we need to rotate to change pitches?
		// Good values are between 10 and 60 degrees
		// Note, that the transition region is often larger than 10 degrees, so if m_flPitchBoundary ends up being close to 10,
		// we'll actually start off the swing in the middle of a transition. This is OK. More variation.
		m_flPitchBoundary1 = (rand() % (int)(m_flMaxPitchBoundaryRads - m_flMinPitchBoundaryRads + 1)) + m_flMinPitchBoundaryRads;

		// The second transition zone is 180 degrees from pitch boundary. This is only useful for spins where we've gone at least 180
		// from the pitch boundary without changing directions.
		m_flPitchBoundary2 = fmod( m_flPitchBoundary1 + M_PI, 2*PI );
	}

	SmoothSwingAmountA 	= m_flSwingPitchLowHiLerp;
	SmoothSwingAmountB 	= 1.0 - m_flSwingPitchLowHiLerp;
}

/******************************************************************************/
/*!
 * @name SmoothSwing V2S
 * @brief  A modified version of SmoothSwingV2
 */
/******************************************************************************/
void MOTION::SmoothSwingV2S(uint32_t nCurrentTime, float gx, float gy, float gz, bool useMicros)
{
    static float m_nLastUpdateTime = 0.0f;  // Inizializza m_nLastUpdateTime
    static float m_flTotalRotationThisSwing = 0.0f;  // Inizializza m_flTotalRotationThisSwing

    float flTimeSinceLastUpdate;

    if (useMicros) flTimeSinceLastUpdate = ( nCurrentTime - m_nLastUpdateTime ) / 1000000.0f;
    else flTimeSinceLastUpdate = ( nCurrentTime - m_nLastUpdateTime ) / 1000.0f;

    m_nLastUpdateTime = nCurrentTime;

    float flTotalAngularVel = sqrtf( powf(gx,2) + powf(gy,2) + powf(gz,2) );

    SwingStrength = min( 1.0f, flTotalAngularVel / SmoothSwingSensitivity );

    m_flTotalRotationThisSwing += flTotalAngularVel * flTimeSinceLastUpdate;

    float flWrappedRotationThisSwing = fmod(m_flTotalRotationThisSwing, 360.0f); // Usare 360 gradi

    // gryo outputs degrees, sinf takes radians
    flWrappedRotationThisSwing = flWrappedRotationThisSwing * (PI / 180.0f);

    SmoothSwingAmountA = (sinf(flWrappedRotationThisSwing) + 1) / 2;
    SmoothSwingAmountB = (sinf(flWrappedRotationThisSwing + PI) + 1) / 2;

}

float MOTION::SmoothSwing_GetSwingStrength()
{
	return SwingStrength;
}

// SmoothSwingAmountA + SmoothSwingAmountB always equal to 1
float MOTION::SmoothSwing_GetSwingA()
{
	return SmoothSwingAmountA;
}

float MOTION::SmoothSwing_GetSwingB()
{
	return SmoothSwingAmountB;
}

// returns value such that:
// SwingAGain + SwingBGain + HumGain <= 1 + SwingOverdrive
// MinGainHum < HumGain < MaxGainHum
// SwingAGain + SwingBGain == MAX_GAIN_SWING, when SwingStrength == 1
float MaxGainHum  = 0.9; // 1 for loudest hum, or something less than 1 so swing sound is comparatively louder.
float MinGainHum  = 0.1;
#define MAX_GAIN_SWING (1.0f + SwingOverdrive)

float SwingOverdrive = 0.1; // for louder swings at the expense of clipping. set depending on font.

// In this case the total volume increase from .7 to 1 over the strength of a swing.
// The percentage of the sound that is hum decreases from 100 to 20 percent.
// The percentage of the sound that is swing increases from  from 0 to 80 percent + SwingOverdrive %.

static unsigned long timeElapsed;

void MOTION::debug_SmoothSwing()
{
	if (millis()-timeElapsed > 100)
	{
		timeElapsed = millis();

		Serial.print("AmountA = ");Serial.print( SmoothSwingAmountA);
		Serial.print("AmountB = ");Serial.print( SmoothSwingAmountB);
		Serial.print("SwingStrength = ");Serial.print( SwingStrength);

		Serial.print("GetSwingAGain = ");Serial.print( SmoothSwing_GetSwingAGain());
		Serial.print("GetSwingBGain = ");Serial.print( SmoothSwing_GetSwingBGain());
		Serial.print("GetHumGain = ");Serial.print( SmoothSwing_GetHumGain());

		Serial.println( );
	}
}



// Definizione delle costanti
static constexpr float SWING_SENSITIVITY = 360;
static constexpr float ROLL_SENSITIVITY = 600;
static constexpr float MAXIMUM_HUM_DUCKING = 0.75;
static constexpr float SWING_SHARPNESS = 5.0;
static constexpr float TRANSITION_1_MIN = 30;
static constexpr float TRANSITION_1_MAX = 50;
static constexpr float TRANSITION_1_WIDTH = 60.0;
static constexpr float TRANSITION_2_WIDTH = 160.0;
static constexpr float MY_SMOOTHING_FACTOR = 0.4;


// Variabili globali
float PrevGyroX, PrevGyroY, PrevGyroZ;
float GyroX, GyroY, GyroZ;
float AccelX, AccelY, AccelZ;
float AngAccX, AngAccY, AngAccZ;
float PrevAngAccX, PrevAngAccY, PrevAngAccZ;
float AngDotProduct;

float totalRotation = 0;
float transitionVolume = 0;
float transition1midPoint = 0;
float transition2midPoint = 0;

float swingSpeed = 0;
float rollSpeed = 0;
float swingStrength = 0;

float prevHumVolume = 0;
float prevSwingVolumeA = 0;
float prevSwingVolumeB = 0;

float humVolume = 0;
float swingVolumeA = 0;
float swingVolumeB = 0;

bool gestureType = 0;

bool swingActive = false;
float finalHumVolume = 0;
float finalSwingVolumeA = 0;
float finalSwingVolumeB = 0;

unsigned long t0, t1, delta;

float test_getSwingSpeed()
{
    return sqrtf(GyroX * GyroX + GyroY * GyroY);
}

float test_getRollSpeed()
{
    return std::abs(GyroZ);
}


void MOTION::Test_SmoothSwing(float ax, float ay, float az, float gx, float gy, float gz)
{
    PrevGyroX = gx;
    PrevGyroY = gy;
    PrevGyroZ = gz;

    GyroX = (MY_SMOOTHING_FACTOR * gx) + (1.0 - MY_SMOOTHING_FACTOR) * GyroX;
    GyroY = (MY_SMOOTHING_FACTOR * gy) + (1.0 - MY_SMOOTHING_FACTOR) * GyroY;
    GyroZ = (MY_SMOOTHING_FACTOR * gz) + (1.0 - MY_SMOOTHING_FACTOR) * GyroZ;

    AccelX = (MY_SMOOTHING_FACTOR * gy * ax) + (1.0 - MY_SMOOTHING_FACTOR * gy) * AccelX;
    AccelY = (MY_SMOOTHING_FACTOR * gy * ay) + (1.0 - MY_SMOOTHING_FACTOR * gy) * AccelY;
    AccelZ = (MY_SMOOTHING_FACTOR * gy * az) + (1.0 - MY_SMOOTHING_FACTOR * gy) * AccelZ;

    float Module = sqrtf(GyroX * GyroX + GyroY * GyroY + GyroZ * GyroZ);

    AngAccX = (GyroX - PrevGyroX) / Module;
    AngAccY = (GyroY - PrevGyroY) / Module;
    AngAccZ = (GyroZ - PrevGyroZ) / Module;

    PrevAngAccX = (MY_SMOOTHING_FACTOR * AngAccX) + (1.0 - MY_SMOOTHING_FACTOR) * PrevAngAccX;
    PrevAngAccY = (MY_SMOOTHING_FACTOR * AngAccY) + (1.0 - MY_SMOOTHING_FACTOR) * PrevAngAccY;
    PrevAngAccZ = (MY_SMOOTHING_FACTOR * AngAccZ) + (1.0 - MY_SMOOTHING_FACTOR) * PrevAngAccZ;
    AngDotProduct = AngAccX * PrevAngAccX + AngAccY * PrevAngAccY + AngAccZ * PrevAngAccZ;

    swingSpeed = test_getSwingSpeed();
    rollSpeed = test_getRollSpeed();

    if (AngDotProduct > 0.2)
    {
        // Reset swing
        t0 = micros();
        totalRotation = 0;
        transitionVolume = 0;
        transition1midPoint = random(TRANSITION_1_MIN, TRANSITION_1_MAX);
        transition2midPoint = transition1midPoint + 180;
    }

    t1 = micros();
    delta = t1 - t0;
    t0 = t1;

    // if (swingSpeed > rollSpeed - 500)
    {
        totalRotation += swingSpeed * delta / 1000000;
        swingStrength = fmin(1, swingSpeed / SWING_SENSITIVITY);
        gestureType = 0;
    }
    /* else
    {
        Serial.println("ROLL");
        totalRotation += rollSpeed * delta / 1000000;
        swingStrength = fmin(1, rollSpeed / ROLL_SENSITIVITY);
        gestureType = 1;
    }
    */

    if (totalRotation > 360)  totalRotation = 0;


    if (totalRotation > (transition1midPoint - TRANSITION_1_WIDTH) && totalRotation < (transition1midPoint + TRANSITION_1_WIDTH)) {
        transitionVolume = (totalRotation - transition1midPoint + TRANSITION_1_WIDTH / 2) / TRANSITION_1_WIDTH;
        transitionVolume = fmax(0, transitionVolume);
        transitionVolume = fmin(1, transitionVolume);
    }

    if (totalRotation > (transition2midPoint - TRANSITION_2_WIDTH) && totalRotation < (transition2midPoint + TRANSITION_2_WIDTH)) {
        transitionVolume = 1 - (totalRotation - transition2midPoint + TRANSITION_2_WIDTH / 2) / TRANSITION_2_WIDTH;
        transitionVolume = fmax(0, transitionVolume);
        transitionVolume = fmin(1, transitionVolume);
    }

    swingStrength = powf(swingStrength, SWING_SHARPNESS);       // Add sharmpess to the swing. Higher value means more pronunced swing
    humVolume = 1.0 - swingStrength * MAXIMUM_HUM_DUCKING;
    swingVolumeA = swingStrength * transitionVolume;
    swingVolumeB = swingStrength * (1 - transitionVolume);
}


float getAngDotProduct()
{
    return AngDotProduct;
}

float getHumVolume()
{
    return humVolume;
}

float getSwingVolumeA()
{
    return swingVolumeA;
}

float getSwingVolumeB()
{
    return swingVolumeB;
}

void MOTION::debug_my_smoothswing()
{
    static uint32_t timeElapsed = 0;

    if (millis() - timeElapsed > 10)
    {
        timeElapsed = millis();
        Serial.print("AdP = ");
        Serial.print(AngDotProduct);
        Serial.print(" S = ");
        Serial.print(swingSpeed,4);
        Serial.print(" H = ");
        Serial.print(getHumVolume(),3);
        Serial.print(" A = ");
        Serial.print(getSwingVolumeA(),3);
        Serial.print(" B = ");
        Serial.print(getSwingVolumeB(),3);
        Serial.println();

        //plot(getHumVolume(),getSwingVolumeA(),getSwingVolumeB());
    }
}

float prevSwingAGain = 0.0f;
float prevSwingBGain = 0.0f;
float prevHumGain = 0.0f;

float max_step_change_allowed = 0.2f;

// Limiti minimi e massimi per i gain di SwingA
float MIN_GAIN_SWING_A = 0.0f;
float MAX_GAIN_SWING_A = 1.0f;

// Limiti minimi e massimi per i gain di SwingB
float MIN_GAIN_SWING_B = 0.0f;
float MAX_GAIN_SWING_B = 1.0f;

// Limiti minimi e massimi per i gain di Hum
float MIN_GAIN_HUM = 0.0f;
float MAX_GAIN_HUM = 0.60f;

// Fattori di smussamento per SwingA, SwingB e Hum
const float alphaSwingA = 0.01f; // Regola questo valore tra 0 e 1
const float alphaSwingB = 0.02f; // Regola questo valore tra 0 e 1
const float alphaHum = 0.03f; // Regola questo valore tra 0 e 1

// Funzione per limitare il cambiamento dei valori di gain
float limitChange(float currentValue, float prevValue, float step) {
    if (currentValue > prevValue + step) {
        return prevValue + step;
    } else if (currentValue < prevValue - step) {
        return prevValue - step;
    } else {
        return currentValue;
    }
}

// Funzione per applicare il filtro esponenziale smussato
float exponentialSmoothing(float newValue, float prevValue, float alpha) {
    return alpha * newValue + (1 - alpha) * prevValue;
}

// Funzione per limitare i valori entro un range
float clamp(float value, float min, float max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

float MOTION::SmoothSwing_GetSwingAGain() {
    float newSwingAGain = SmoothSwingAmountA * SwingStrength * MAX_GAIN_SWING_A;

    newSwingAGain = limitChange(newSwingAGain, prevSwingAGain, max_step_change_allowed);

    // Applica il filtro esponenziale smussato
    prevSwingAGain = exponentialSmoothing(newSwingAGain, prevSwingAGain, alphaSwingA);

    // Limita il valore tra MIN_GAIN_SWING_A e MAX_GAIN_SWING_A
    prevSwingAGain = clamp(prevSwingAGain, MIN_GAIN_SWING_A, MAX_GAIN_SWING_A);

    return prevSwingAGain;
}

float MOTION::SmoothSwing_GetSwingBGain() {
    float newSwingBGain = SmoothSwingAmountB * SwingStrength * MAX_GAIN_SWING_B;

    newSwingBGain = limitChange(newSwingBGain, prevSwingBGain, max_step_change_allowed);

    // Applica il filtro esponenziale smussato
    prevSwingBGain = exponentialSmoothing(newSwingBGain, prevSwingBGain, alphaSwingB);

    // Limita il valore tra MIN_GAIN_SWING_B e MAX_GAIN_SWING_B
    prevSwingBGain = clamp(prevSwingBGain, MIN_GAIN_SWING_B, MAX_GAIN_SWING_B);

    return prevSwingBGain;
}

float MOTION::SmoothSwing_GetHumGain() {
    float newHumGain = MIN_GAIN_HUM + (1 - SmoothSwingAmountA * SwingStrength - SmoothSwingAmountB * SwingStrength) * (MAX_GAIN_HUM - MIN_GAIN_HUM);

    newHumGain = limitChange(newHumGain, prevHumGain, max_step_change_allowed);

    // Applica il filtro esponenziale smussato
    prevHumGain = exponentialSmoothing(newHumGain, prevHumGain, alphaHum);

    // Limita il valore tra MIN_GAIN_HUM e MAX_GAIN_HUM
    prevHumGain = clamp(prevHumGain, MIN_GAIN_HUM, MAX_GAIN_HUM);

    return prevHumGain;
}
