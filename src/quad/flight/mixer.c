
#include <stdio.h>				// debugging purposes
#include "mixer.h"
#include "pwm_output.h"
#include "rx.h"
#include "maths.h"
#include "runtime_config.h"

static uint8_t motorCount;
static float motorMixRange;

int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

mixerMode_e currentMixerMode;

static motorMixer_t *customMixers;

static motorConfig_t *motorConfig;
static mixerConfig_t *mixerConfig;
rxConfig_t *rxConfig;

static uint16_t disarmMotorOutput;
static float rcCommandThrottleRange;
uint16_t motorOutputHigh, motorOutputLow;

mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];

/* throttle, roll, pitch, yaw
 * 
 * / \
 *  |
 *  |
 *	|  4CW   2CCW
 *  |    \    /
 *  |     \  /
 *  |      \/
 *  |      /\
 *  |     /  \
 *  |    /    \
 *  |  3CCW   1CW
 *	|____________________________>
 * 
 * throttle values for all four motors are always 1.0f
 * For yaw, CW = -1.0f, CCW = 1.0f
 * 
 * See notes <<Motor Mixer>>
 */
static const motorMixer_t mixerQuadX[] = {
	{ 1.0f, -1.0f, 1.0f, -1.0f },				// REAR_RIGHT MOTOR 	(MOTOR 1)
	{ 1.0f, -1.0f, -1.0f, 1.0f },				// FRONT_RIGHT MOTOR	(MOTOR 2)
	{ 1.0f, 1.0f, 1.0f, 1.0f },					// REAR_LEFT MOTOR		(MOTOR 3)
	{ 1.0f, 1.0f, -1.0f, -1.0f },				// FRONT_LEFT MOTOR		(MOTOR 4)
};

void mixerUseConfigs(motorConfig_t *motorConfigToUse, mixerConfig_t *mixerConfigToUse, rxConfig_t *rxConfigToUse)
{
	motorConfig = motorConfigToUse;
	mixerConfig = mixerConfigToUse;
	rxConfig = rxConfigToUse;
}

void mixerResetDisarmedMotors(void)
{
	/* set disarmed motor values */
	for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
		motor_disarmed[i] = disarmMotorOutput;			// disarmMotorOutput = motorConfig->mincommand = 1000
	}
}

void mixerConfigurationOutput(void)
{
	motorCount = QUAD_MOTOR_COUNT;
	
	for (int i = 0; i < motorCount; i++) {
		currentMixer[i] = mixerQuadX[i];
	}
	
	mixerResetDisarmedMotors();
}

uint8_t getMotorCount(void)
{
	return motorCount;
}

float getMotorMixRange(void)
{
    return motorMixRange;
}

void writeMotors(void)
{
	if (pwmAreMotorsEnabled()) {
		for (int i = 0; i < motorCount; i++) {
#if 0
			if (i == 0)
				printf("motor 1 PWM control value: %u\r\n", motor[i]);
			else if (i == 1)
				printf("motor 2 PWM control value: %u\r\n", motor[i]);
			else if (i == 2)
				printf("motor 3 PWM control value: %u\r\n", motor[i]);
			else if (i == 3) {
				printf("motor 4 PWM control value: %u\r\n", motor[i]);
				printf("\r\n");
			}
#endif
			pwmWriteMotor(i, motor[i]);
		}
	}
	
	pwmCompleteMotorUpdate(motorCount);
}

bool isMotorProtocolDshot(void)
{
#ifdef USE_DSHOT
	switch (motorConfig->motorPwmProtocol) {
		case PWM_TYPE_DSHOT1200:
		case PWM_TYPE_DSHOT600:
		case PWM_TYPE_DSHOT300:
		case PWM_TYPE_DSHOT150:
			return true;
		default:
			return false;
	}
#else
	return false;
#endif
}

/* Scaled ESC outputs */
void initEscEndpoints(void)
{
#ifdef USE_DSHOT
	/* TODO: DSHOT Esc initialisation implementation here */
	if (isMotorProtocolDshot()) {
		
	}else
#endif
	{
		disarmMotorOutput = motorConfig->mincommand;				// motorConfig->mincommand = 1000
		motorOutputLow = motorConfig->minthrottle;					// motorConfig->minthrottle = 1045 or 1070
		motorOutputHigh = motorConfig->maxthrottle;					// motorConfig->maxthrottle = 2000
//		printf("disarmMotorOutput: %u\r\n", disarmMotorOutput);		// disarmMotorOutput = 1000
//		printf("motorOutputLow: %u\r\n", motorOutputLow);			// motorOutputLow = 1070
//		printf("motorOutputLow: %u\r\n", motorOutputHigh);			// motorOutputHigh = 2000
	}
	
	rcCommandThrottleRange = (PWM_RANGE_MAX - rxConfig->mincheck);	// PWM_RANGE_MAX = 2000, rxConfig->mincheck = 1100, rcCommandThrottleRange = 2000 - 1100 = 900
//	printf("rcCommandThrottleRange: %f\r\n", rcCommandThrottleRange);	// rcCommandThrottleRange = 900.000000
}

void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers)
{
	currentMixerMode = mixerMode;			// mixerMode = MIXER_QUADX = 3
	customMixers = initialCustomMixers;
	initEscEndpoints();
}

void mixTable(pidProfile_t *pidProfile)
{
	/* Scale roll/pitch/yaw uniformly to fit within throttle range */
	float throttle, currentThrottleInputRange = 0;
	uint16_t motorOutputMin, motorOutputMax;
	static uint16_t throttlePrevious = 0;			// store the last throttle direction for deadband transitions
	bool mixerInversion = false;					// ONLY use this variable in 3D flight and DSHOT motor protocols

	/* +-------------------------------------------------------------------------------------------------+ */
	/* +------------------ Find Min and Max throttle values based on current condition ------------------+ */
	/* +-------------------------------------------------------------------------------------------------+ */
	
//	printf("rcCommand[THROTTLE]: %d, %s, %d\r\n", rcCommand[THROTTLE], __FUNCTION__, __LINE__);
	throttle = rcCommand[THROTTLE] - rxConfig->mincheck;	// current rcCommand[THROTTLE] value - 1100 (mincheck)
	currentThrottleInputRange = rcCommandThrottleRange;		// rcCommandThrottleRange = 2000 - 1100 = 900
//	printf("throttle: %f\r\n", throttle);		// throttle = [-100.000;892]
//	printf("currentThrottleInputRange: %f, %s, %d\r\n", currentThrottleInputRange, __FUNCTION__, __LINE__);
	
	/* Find min and max throttle based on condition */
	motorOutputMax = motorOutputHigh;		// motorOutputMax = motorOutputHigh = 2000 (max throttle)
	motorOutputMin = motorOutputLow;		// motorOutputMin = motorOutputMin = 1070 (min throttle)
	
	/* throttle / currentThrottleInputRange = [-0.1111;0.992222] */
//	printf("throttle / currentThrottleInputRange: %f, %s, %d\r\n", throttle / currentThrottleInputRange, __FUNCTION__, __LINE__);
    
    /* throttle / currentThrottleInputRange is between -0.111111(-100/900) and 0.986667( (1990-1100)/900 )
     * throttle is between 0.0 and 1.0
     */
	throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);	// currentThrottleInputRange = 2000 - 1100 = 900
//	printf("throttle after constrainf: %f\r\n", throttle);
	
	const float motorOutputRange = motorOutputMax - motorOutputMin;		// motorOutputMax - motorOutputMin = 2000 - 1070 = 930
//	printf("motorOutputRange: %f\r\n", motorOutputRange);				// 930

	/* +----------------------------------------------------------------------------------------------------+ */
	/* +---------------------------------- Calculate and limit the PIDsum ----------------------------------+ */
	/* +----------------------------------------------------------------------------------------------------+ */
	float scaledAxisPIDf[3];
	
//	printf("ROLLSum: %f\r\n", axisPID_P[FD_ROLL] + axisPID_I[FD_ROLL] + axisPID_D[FD_ROLL]);
//	printf("PITCHSum: %f\r\n", axisPID_P[FD_PITCH] + axisPID_I[FD_PITCH] + axisPID_D[FD_PITCH]);
//	printf("YAWSum: %f\r\n", axisPID_P[FD_YAW] + axisPID_I[FD_YAW] + axisPID_D[FD_YAW]);
	
	/* pidProfile->pidSumLimit = 0.5f */
	scaledAxisPIDf[FD_ROLL] = 
		constrainf((axisPID_P[FD_ROLL] + axisPID_I[FD_ROLL] + axisPID_D[FD_ROLL]) / PID_MIXER_SCALING, -pidProfile->pidSumLimit, pidProfile->pidSumLimit);
	
	scaledAxisPIDf[FD_PITCH] = 
		constrainf((axisPID_P[FD_PITCH] + axisPID_I[FD_PITCH] + axisPID_D[FD_PITCH]) / PID_MIXER_SCALING, -pidProfile->pidSumLimit, pidProfile->pidSumLimit);
		
	scaledAxisPIDf[FD_YAW] = 
		constrainf((axisPID_P[FD_YAW] + axisPID_I[FD_YAW] + axisPID_D[FD_YAW]) / PID_MIXER_SCALING, -pidProfile->pidSumLimit, pidProfile->pidSumLimitYaw);
	
	/* +----------------------------------------------------------------------------------------------------+ */
	/* +---------------------------------- Calculate voltage compensation ----------------------------------+ */
	/* +----------------------------------------------------------------------------------------------------+ */
//	const float vbatCompensationFactor = (BatteryConfig && pidProfile->vbatPidCompensation) ? calculateVbatPidCompensation() : 1.0f;
	const float vbatCompensationFactor = 1.0f;					// TODO: implement later
	
	/* +----------------------------------------------------------------------------------------------------+ */
	/* +-------------------------------- Find Roll/Pitch/Yaw desired outputs -------------------------------+ */
	/* +----------------------------------------------------------------------------------------------------+ */
	float motorMix[MAX_SUPPORTED_MOTORS];
	float motorMixMax = 0, motorMixMin = 0;
	
	/* mixerConfig->yaw_motor_direction = 1
	 *     thr   rol    pit    yaw
	 * 	{ 1.0f, -1.0f, 1.0f, -1.0f },				// REAR_RIGHT MOTOR 	(MOTOR 1)
	 *  { 1.0f, -1.0f, -1.0f, 1.0f },				// FRONT_RIGHT MOTOR	(MOTOR 2)
	 *  { 1.0f, 1.0f, 1.0f, 1.0f },					// REAR_LEFT MOTOR		(MOTOR 3)
	 *  { 1.0f, 1.0f, -1.0f, -1.0f },				// FRONT_LEFT MOTOR		(MOTOR 4)
	 */
	for (int i = 0; i < motorCount; i++) {
//		printf("currentMixer[%d]: [%f, %f, %f, %f]\r\n", i, currentMixer[i].throttle, currentMixer[i].roll, currentMixer[i].pitch, currentMixer[i].yaw);
		motorMix[i] = scaledAxisPIDf[PITCH] * currentMixer[i].pitch + scaledAxisPIDf[ROLL] * currentMixer[i].roll + 
					  scaledAxisPIDf[YAW] * currentMixer[i].yaw * (-mixerConfig->yaw_motor_direction);
		
//		printf("motor[%d]: %f\r\n\r\n", i, motorMix[i]);
		
		/* Add voltage compensation */
		if (vbatCompensationFactor > 1.0f) {
			motorMix[i] *= vbatCompensationFactor;
		}
		
//		printf("motorMix[%d]: %f\r\n", i, motorMix[i]);
//		printf("motorMixMax: %f\r\n", motorMixMax);
		
		if (motorMix[i] > motorMixMax) {
			motorMixMax = motorMix[i];
//			printf("motorMixMax: %f\r\n", motorMixMax);
		} else if (motorMix[i] < motorMixMin) {
			motorMixMin = motorMix[i];
//			printf("motorMixMin: %f\r\n", motorMixMin);
		}
	}
	
//	printf("(min,max): (%f,%f)\r\n", motorMixMin, motorMixMax);
	/* Calculate the motorMixRange to check if motors are saturated or not */
	/* motorMixRange ranges from 0.00xxx to 2.00000
	 * 2.0000 when moving roll, pitch stick ONLY to the top-left, top-right, bottom-left and bottom-right corners, 
	 * 1.0000 when moving throttle, yaw stick ONLY to the top-left, top-right, bottom-left and bottom-right corners
	 * 0.00xx when roll, pitch stick at the centre position, yaw at the centre position and throttle at the lowest position.
	 */
	motorMixRange = motorMixMax - motorMixMin;
//	printf("motorMixRange: %f\r\n", motorMixRange);
	
	/* TODO: DELETE later, for motor calibration of F450 quadcopter */
//	for (uint32_t i = 0; i < motorCount; i++) {
////		motor[i] = motorOutputMin;
////		printf("motor[%d]: %d\r\n", i, motor[i]);
//		motor[i] = rcCommand[THROTTLE];
//	}
	
	if (motorMixRange > 1.0f) {
		for (int i = 0; i < motorCount; i++) {
			motorMix[i] /= motorMixRange;
//			printf("motorMix[%d]: %f\r\n", i, motorMix[i]);
		}
		
		/* Get the maximum correction by setting offset to centre when airmode is enabled */
		if (isAirModeActive()) {
			throttle = 0.5f;
		}
	} else {
		/* Only automatically adjust throttle during airmode scenario */
		if (isAirModeActive()) {
//			printf("motorMixRange: %f\r\n", motorMixRange);
			float throttleLimitOffset = motorMixRange / 2.0f;
//			printf("throttle1: %f\r\n", throttle);
//			printf("throttleLimitOffset: %f\r\n", throttleLimitOffset);
			throttle = constrainf(throttle, 0.0f + throttleLimitOffset, 1.0f - throttleLimitOffset);
		}
	}

//	printf("throttle2: %f\r\n", throttle);
	
	uint32_t i = 0;
	
	/* +----------------------------------------------------------------------------------------------------+ */
	/* +----------------------- Calculate the desired motors' output values when ARMed ---------------------+ */
	/* +----------------------------------------------------------------------------------------------------+ */
	for (uint32_t i = 0; i < motorCount; i++) {
//		motor[i] = motorOutputMin;
//		printf("motor[%d]: %d\r\n", i, motor[i]);
//		motor[i] = rcCommand[THROTTLE];
//		printf("motorOutputMin: %u\r\n", motorOutputMin);				// motorOutputMin = minthrottle = 1070
//		printf("motorOutputRange: %f\r\n", motorOutputRange);			// motorOutputRange = motorOutputMax - motorOutputMin = 2000 - 1070 = 930
//		printf("motorMix[%d]: %f\r\n", i, motorMix[i]);					// see motor output notes

		/*
		 * THREE scenarios when testing on the bench WITHOUT props on:
		 * Case 1: When throttle stick at the lowest position, the throttle value is close to 0.00 (0.00xxx or something)
		 * Case 2: When moving throttle stick up a little bit or any position other than the lowest one, the throttle value
		 *         will be gradually increasing. The reason for this is because the PID controller is trying to compensate
		 *         the random vibrations coming from the motors, since the motors don't have props on, the PID controller doesn't
		 *		   have any effect, therefore, it (PID controller) keeps trying harder and harder (i.e. drive the motors spin faster and faster) to 
		 *         feedback to the controller to get rid of the vibration, which results in the motor spinning faster and faster.
		 * Case 3: The maximum throttle value while moving Throttle, Roll, Pitch and Yaw sticks is 0.5 
		 */
//		printf("throttle: %f\r\n", throttle);
//		printf("currentMixer[%d].throttle: %f\r\n", i, currentMixer[i].throttle);		// all 1.0f
//		printf("thr+mix[%d]: %f\r\n", i, motorMix[i] + (throttle * currentMixer[i].throttle));
//		printf("range*(thr+mix[%d]): %f\r\n", i, motorOutputRange * (motorMix[i] + (throttle * currentMixer[i].throttle)));
//		printf("lrintf(range*(thr+mix[%d])): %ld\r\n", i, lrintf(motorOutputRange * (motorMix[i] + (throttle * currentMixer[i].throttle))));
		/* lrintf rounds the floating-point number to an integer value according to the current round mode */
		motor[i] = motorOutputMin + lrintf(motorOutputRange * (motorMix[i] + (throttle * currentMixer[i].throttle)));	// motorOutputMin = 1070 (minthrottle)
		printf("motorNew[%d]: %d\r\n", i, motor[i]);		// motor[i] outputs values are based on the minthrottle (1045 or 1070) setup in config.c
		
		/* DSHOT works exactly opposite in lower 3D section */
		if (mixerInversion) {
			motor[i] = motorOutputMin + (motorOutputMax - motor[i]);
		}

		/* TODO: Implement failsafe feature later */
//		if (failsafeIsActive()) {
//			if (isMotorProtocolDshot()) {
//				/* Prevent getting into special reserved range */
//				motor[i] = (motor[i] < motorOutputMin) ? disarmMotorOutput : motor[i];
//			}
//			
//			motor[i] = constrain(motor[i], disarmMotorOutput, motorOutputMax);
//		} else
		{
			/*
			 * motorOutputMin = minthrottle = 1045 in current case.
			 * motorOutputMax = maxthrottle = 2000
			 *
			 * minthrottle(1045) <= motor[i] <= 2000
			 */
			motor[i] = constrain(motor[i], motorOutputMin, motorOutputMax);
		}
		
		/* Motor stop handler */
		if (feature(FEATURE_MOTOR_STOP) && CHECK_ARMING_FLAG(ARMED) && !isAirModeActive()) {
			if (rcData[THROTTLE] < rxConfig->mincheck) {
				motor[i] = disarmMotorOutput;		// disarmMotorOutput = mincommand = 1000
			}
		}
	}
	
	/** +---------------------------------------------------------------------------------------------------+
	 *  +----------------------------------------- DISARMed mode -------------------------------------------+
	 *  +---------------------------------------------------------------------------------------------------+
	 * Assign the motors' output values to disarmMotorOutput(1000) when DISARMed.
	 *
	 * If ARMING_FLAG is NOT set to ARMED, meaning DISARMED, no matter what value assigned to
	 * the motor[i] output previously, it will be replaced by disarmMotorOutput value (1000)
     * in this case.	
	 */
	if (!CHECK_ARMING_FLAG(ARMED)) {
//		printf("disarmMotorOutput: %u\r\n", disarmMotorOutput);
		for (i = 0; i < motorCount; i++) {
			motor[i] = disarmMotorOutput;		// disarmMotorOutput = MotorConfig()->mincommand = 1000
		}
	}
	
//	if (IS_RC_MODE_ACTIVE(BOXARM)) {
//		/* TODO: modify this when finishing PID controllers */
//		for (uint32_t i = 0; i < motorCount; i++) {
//	//		motor[i] = motorOutputMin;
//	//		printf("motor[%d]: %d\r\n", i, motor[i]);
//			motor[i] = rcCommand[THROTTLE];
//		}
//	} else {
//		for (uint32_t i = 0; i < motorCount; i++) {
//	//		motor[i] = motorOutputMin;
//	//		printf("motor[%d]: %d\r\n", i, motor[i]);
//			motor[i] = 1000;
//		}
//	}
}
