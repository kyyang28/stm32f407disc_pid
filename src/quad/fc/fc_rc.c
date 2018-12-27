
#include <stdio.h>				// debugging purpose
#include "rx.h"
#include "configMaster.h"		// currentControlRateProfile, currentProfile, masterConfig
#include "maths.h"
#include "fc_core.h"
#include "scheduler.h"
#include "rc_controls.h"
#include "runtime_config.h"		// armingFlags, flightModeFlags, ENABLE_FLIGHT_MODE, DISABLE_FLIGHT_MODE, FLIGHT_MODE
#include "pid.h"

#define THROTTLE_LOOKUP_LENGTH			12

/* Variables for setting setpoint */
#define RC_RATE_INCREMENTAL				14.54f
#define SETPOINT_RATE_LIMIT				1998.0f

static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3];
static float throttlePIDAttenuation;

static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];		// lookup table for expo & mid THROTTLE

float getSetpointRate(int axis)
{
	return setpointRate[axis];
}

float getRcDeflection(int axis)
{
	return rcDeflection[axis];
}

float getRcDeflectionAbs(int axis)
{
	return rcDeflectionAbs[axis];
}

float getThrottlePIDAttenuation(void)
{
	return throttlePIDAttenuation;
}

/* setup throttle curve and fill up the lookupThrottleRC table array */
void generateThrottleCurve(void)
{	
	for (uint8_t i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
		int16_t tmp = 10 * i - currentControlRateProfile->thrMid8;		// thrMid8 = 50
		uint8_t y = 1;
		
		if (tmp > 0) {
			y = 100 - currentControlRateProfile->thrMid8;
		}
		
		if (tmp < 0) {
			y = currentControlRateProfile->thrMid8;
		}
		
		/*
		 * if thrMid8 = 50 && thrExpo8 = 0
		 * then
		 * 	lookupThrottleRC[i] = 0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100
		 */
		lookupThrottleRC[i] = 10 * currentControlRateProfile->thrMid8 + tmp * (100 - currentControlRateProfile->thrExpo8 + (int32_t)currentControlRateProfile->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
//		printf("lookupThrottleRC[%d] before: %d\r\n", i, lookupThrottleRC[i]);

		/*
		 * if thrMid8 = 50 && thrExpo8 = 0
		 * then
		 * 	lookupThrottleRC[i] = 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100
		 */
		lookupThrottleRC[i] = PWM_RANGE_MIN + (PWM_RANGE_MAX - PWM_RANGE_MIN) * lookupThrottleRC[i] / 1000;		// [MIN_THROTTLE;MAX_THROTTLE]
//		printf("lookupThrottleRC[%d] after: %d\r\n", i, lookupThrottleRC[i]);
	}
}

static int16_t rcLookupThrottle(int32_t tmp)
{
	const int32_t l_tmp = tmp / 100;
	
	/* [0;1000] -> expo -> [MIN_THROTTLE;MAX_THROTTLE] */
	return lookupThrottleRC[l_tmp] + (tmp - l_tmp * 100) * (lookupThrottleRC[l_tmp + 1] - lookupThrottleRC[l_tmp]) / 100;
}

void updateRcCommands(void)
{
	/* PITCH & ROLL only dynamic PID adjustment, depending on throttle value */
	int32_t prop;
	
//	printf("Current throttle value: %d\r\n", rcData[THROTTLE]);
//	printf("Throttle PID Attenuation(TPA) breakpoint value: %u\r\n", currentControlRateProfile->tpa_breakpoint);
//	printf("Maximum PID attenuation value based on throttle: %u%%\r\n", currentControlRateProfile->dynThrPID);
	
	/* +-----------------------------------------------------------------------------------------------+ */
	/* +------------------------------------------- TPA part ------------------------------------------+ */
	/* +-----------------------------------------------------------------------------------------------+ */
	/* calculate the throttlePIDAttenuation value */
	if (rcData[THROTTLE] < currentControlRateProfile->tpa_breakpoint) {
		prop = 100;
		throttlePIDAttenuation = 1.0f;
//		printf("Current throttle value is less than TPA (utilising 100%% PID values): %.2f\r\n\r\n", throttlePIDAttenuation);
	}else {
		if (rcData[THROTTLE] < 2000) {		// tpa_breakpoint <= rcData[THROTTLE] <= 2000
			prop = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
//			printf("%u%% PID values are attenuated!!\r\n", 100 - prop);
		} else {		// rcData[THROTTLE] >= 2000, reach the maximum attenuated TPA percentage value
			prop = 100 - currentControlRateProfile->dynThrPID;
//			printf("Maximum %u%% PID values are attenuated!!\r\n", 100 - prop);
		}
//		printf("PID percentage after throttle attenuation %d%%\r\n", prop);
		throttlePIDAttenuation = prop / 100.0f;		// make sure to use 100.0f to get float value of throttlePIDAttenuation, otherwise it will just be 0.00 all the time
//		printf("Current throttle value is greater than TPA (utilising %d%% PID values): %.2f\r\n\r\n", prop, throttlePIDAttenuation);
	}
	
	/* +-------------------------------------------------------------------------------------------------+ */
	/* +------------------------------------- ROLL, PITCH & YAW part ------------------------------------+ */
	/* +-------------------------------------------------------------------------------------------------+ */
	/* interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
	 * 
	 * 								THROTTLE
	 *     1000                       1500                        2000
	 *     
	 *						    ROLL, PITCH & YAW
	 *     1000                       1500                        2000
	 *     -500                        0                          +500
	 */
	for (int axis = 0; axis < 3; axis++) {
		/* non-coupled PID reduction scaler used in PID controller 1 and PID controller 2 */
//		printf("rcData[%d]: %d\r\n", axis, rcData[axis]);
		int32_t tmp = MIN(ABS(rcData[axis] - RxConfig()->midrc), 500);
//		printf("tmp[%d]: %d\r\n", axis, tmp);
		
		/* ROLL = 0, PITCH = 1 */
		if (axis == ROLL || axis == PITCH) {
			if (tmp > RcControlsConfig()->deadband) {		// RcControlsConfig()->deadband = 0 (default initial value)
				tmp -= RcControlsConfig()->deadband;		// non centre position needs to trim the deadband value by deduction
			}else {			// tmp = 0 which means the rcData[ROLL] or rcData[PITCH] is in mid position (1500)
				tmp = 0;
			}
			
			rcCommand[axis] = tmp;		// assign ROLL and PITCH stick values to their corresponding rcCommand
//			printf("rcCommand[%d]: %d\r\n", axis, rcCommand[axis]);
		
		}else {		// YAW case
			if (tmp > RcControlsConfig()->yaw_deadband) {
				tmp -= RcControlsConfig()->yaw_deadband;
			}else {
				tmp = 0;
			}
			
			rcCommand[axis] = tmp * -RcControlsConfig()->yaw_control_direction;		// RcControlsConfig()->yaw_control_direction initial value is 1
//			printf("rcCommand[%d]: %d\r\n", axis, rcCommand[axis]);
		}
		
		if (rcData[axis] < RxConfig()->midrc) {
			rcCommand[axis] = -rcCommand[axis];
		}
	}
	
	/* +-----------------------------------------------------------------------------------------------+ */
	/* +---------------------------------------- THROTTLE part ----------------------------------------+ */
	/* +-----------------------------------------------------------------------------------------------+ */	
	int32_t tmp;
	
	/*
	 * if rcData[THROTTLE] < mincheck, tmp = mincheck (1100)
	 * else if rcData[THROTTLE] > PWM_RANGE_MAX, tmp = PWM_RANGE_MAX (2000)
	 * else tmp = rcData[THROTTLE]
	 */
	tmp = constrain(rcData[THROTTLE], RxConfig()->mincheck, PWM_RANGE_MAX);		// mincheck = 1100, PWM_RANGE_MAX = 2000
//	printf("constrained THROTTLE value: %d\r\n", tmp);			// range [1100;2000], 1100 = mincheck
	tmp = (uint32_t)(tmp - RxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - RxConfig()->mincheck);		// PWM_RANGE_MIN = 1000
//	printf("converted THROTTLE value: %d\r\n", tmp);			// range [0;1000]
	
	/* assign throttle stick value to rcCommand[THROTTLE] */
	rcCommand[THROTTLE] = rcLookupThrottle(tmp);
	
	/* rcCommand[THROTTLE] value is slightly smaller than actual THROTTLE value.
	 * e.g.  actual throttle value is contrained between 1100 (mincheck) and 2000 (pwm max value)
	 *
	 * @1100, rcCommand[THROTTLE] = 1000
	 * @1167, rcCommand[THROTTLE] = 1074
	 * @1200, rcCommand[THROTTLE] = 1111
	 * @1491, rcCommand[THROTTLE] = 1434
	 */
//	printf("rcCommand[THROTTLE]: %d, %s, %d\r\n", rcCommand[THROTTLE], __FUNCTION__, __LINE__);			// range [0;1000]
}

/* Implement later for ANTI-GRAVITY mode */
static void checkForThrottleErrorResetState(uint16_t rxRefreshRate)
{
	
}

/* Calculate the setpoint rate for each axis (ROLL, PITCH and YAW) */
static void calculateSetpointRate(int axis)
{
	uint8_t rcExpo;
	float rcRate;
	
	if (axis != YAW) {
		/* Handle ROLL and PITCH */
		rcExpo = currentControlRateProfile->rcExpo8;				// rcExpo = rcExpo8 = 0
		rcRate = currentControlRateProfile->rcRate8 / 100.0f;		// rcRate = rcRate8 / 100.0f = 100 / 100.0f = 1.0f
	} else {
		/* Handle YAW */
		rcExpo = currentControlRateProfile->rcYawExpo8;				// rcExpo = rcYawExpo8 = 0
		rcRate = currentControlRateProfile->rcYawRate8 / 100.0f;	// rcRate = rcYawRate8 / 100.0f = 100 / 100.0f = 1.0f
	}
	
	if (rcRate > 2.0f) {
		/*
		 * RC_RATE_INCREMENTAL = 14.54f
		 * SETPOINT_RATE_LIMIT = 1998.0f
		 */
		rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
	}
	
//	printf("rcCommand[%d]: %d\r\n", axis, rcCommand[axis]);
	/** rcCommandf values for each axis
	 *
	 *			left		mid			right
	 * ROLL	   -1.0f   to	0.0f   to	+1.0f
	 *
	 * 			top			mid			bottom
     * PITCH   +1.0f   to	0.0f   to	-1.0f	
	 *
	 * 			left		mid			right
     * YAW     +1.0f   to	0.0f   to	-1.0f	
	 */
	float rcCommandf = rcCommand[axis] / 500.0f;
	
	/** rcDefelction array stores the rcCommandf values for each axis
	 *
     *			left		mid			right
	 * ROLL	   -1.0f   to	0.0f   to	+1.0f
	 *
	 * 			top			mid			bottom
     * PITCH   +1.0f   to	0.0f   to	-1.0f	
	 *
	 * 			left		mid			right
     * YAW     +1.0f   to	0.0f   to	-1.0f	
	 */
	rcDeflection[axis] = rcCommandf;
//	printf("rcDeflection[%d]: %f\r\n", axis, rcDeflection[axis]);
	
	/* rcCommandf value ranges from -500 to +500
	 * rcCommandAbs value takes the absolute value of rcCommandf
	 */
	const float rcCommandAbs = ABS(rcCommandf);
	rcDeflectionAbs[axis] = rcCommandAbs;
	
	if (rcExpo) {
		const float expof = rcExpo / 100.0f;
//		printf("rcCommandAbs: %f\r\n", rcCommandAbs);		// rcCommandAbs ranges from 1.0f to 0.0f to 1.0f, always positive number.
		/* rcCommandf is original rcCommand[axis] / 500.0f, which ranges from -1.0f to 0.0f to 1.0f or vice versa. */
		rcCommandf = rcCommandf * POWER3(rcCommandAbs) * expof + rcCommandf * (1 - expof);
//		if (axis == 0)
//			printf("rcCommandf0: %f\r\n", rcCommandf);
//		else if (axis == 1)
//			printf("rcCommandf1: %f\r\n", rcCommandf);
//		else if (axis == 2)
//			printf("rcCommandf2: %f\r\n", rcCommandf);
	}
	
	/* For example, rcRate = 1.0f, rcCommandf = 0.3 (150 [1700 rcData value])
	 *
	 * angleRate = 200.0f * rcRate * rcCommandf = 200.0f * 1.0f * 0.3 = 60.0f
	 */
	float angleRate = 200.0f * rcRate * rcCommandf;
	
	/* currentControlRateProfile->rates[axis] = 70 for each axis (ROLL, PITCH, YAW) */
	if (currentControlRateProfile->rates[axis]) {
		
		/* For example, rates[ROLL] = rates[PITCH] = rates[YAW] = 70, rcCommandf = 0.3, rcCommandAbs = ABS(rcCommandf) = 0.3
		 * rcSuperFactor = 1.0f / constrainf((1.0f - 0.3f * 70 / 100), 0.01f, 1.00f) = 1.0f / constrainf((1.0f - 0.21f), 0.01f, 1.00f)
		 *				 = 1.0f / constrainf(0.79f, 0.01f, 1.00f) = 1.0f / 0.79f = 1.2658227848101265822784810126582
		 */
		const float rcSuperFactor = 1.0f / (constrainf(1.0f - (rcCommandAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));

		/* angleRate = angleRate * rcSuperFactor = 60.0f * 1.2658227848101265822784810126582 = 75.949367088607594936708860759492 */
		angleRate *= rcSuperFactor;
	}
	
	/* SETPOINT_RATE_LIMIT = 1998.0f
	 *
	 * angleRate should be within [-1998.0f, 1998.0f]
	 *
	 * angleRate 75.949367088607594936708860759492 is calculated as above.
	 */
	setpointRate[axis] = constrainf(angleRate, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);
	
//	printf("setpointRate[%d]: %f\r\n", axis, setpointRate[axis]);
}

/* Just for Angle and Horizon modes */
static void scaleRcCommandToFpvCamAngle(void)
{
	
}

void processRcCommand(void)
{
	static uint16_t currentRxRefreshRate;
	static int16_t factor, rcInterpolationFactor;
	static int16_t lastCommand[4] = { 0, 0, 0, 0 };
	static int16_t deltaRC[4] = { 0, 0, 0, 0 };
	const uint8_t interpolationChannels = RxConfig()->rcInterpolationChannels + 2;	// config->rxConfig.rcInterpolationChannels = 0; interpolationChannels = 0 + 2 = 2
	bool readyToCalculateRate = false;
	uint8_t readyToCalculateRateAxisCnt = 0;
	uint16_t rxRefreshRate;
	
	/* isRXDataNew is set to TRUE in taskUpdateRxMain() task function */
	if (isRXDataNew) {
		/* Get the task runtime difference between the current time and last time
		 *
		 * currentRxRefreshRate is between 50Hz and 1KHz
		 */
		currentRxRefreshRate = constrain(getTaskDeltaTime(TASK_RX), 1000, 20000);		// units in microseconds (us)
		
		if (isAntiGravityModeActive()) {
			checkForThrottleErrorResetState(currentRxRefreshRate);
		}
	}
	
	/*
	 * config->rxConfig.rcInterpolation = RC_SMOOTHING_AUTO;	rxConfig.rcInterpolation = 2
	 */
	if (RxConfig()->rcInterpolation || flightModeFlags) {
		/* Set RC refresh rate for sampling and channels to filter */
		switch (RxConfig()->rcInterpolation) {
			case RC_SMOOTHING_AUTO:
//				printf("currentRxRefreshRate: %u\r\n", currentRxRefreshRate);		// currentRxRefreshRate ~= 9000
				rxRefreshRate = currentRxRefreshRate + 1000;		// Add slight overhead to prevent ramps
				break;
			
			case RC_SMOOTHING_MANUAL:
				/* config->rxConfig.rcInterpolationInterval = 19 */
				rxRefreshRate = 1000 * RxConfig()->rcInterpolationInterval;		// rxRefreshRate = 1000 * 19 = 19000
				break;
			
			case RC_SMOOTHING_OFF:
			case RC_SMOOTHING_DEFAULT:
			default:
				rxRefreshRate = rxGetRefreshRate();
		}
		
//		printf("rxRefreshRate: %u\r\n", rxRefreshRate);		// rxRefreshRate ~= 10000
		
		/*
		 * IMPORTANT: As PID looptime is faster than the Rx data receiving.
		 * rcInterpolation is a way to feed in the rx data to the PID loop more smoothly whenever there is no data coming from the transmitter. 
		 */
		if (isRXDataNew) {
//			printf("targetPidLooptime: %u\r\n", targetPidLooptime);		// targetPidLooptime = 500
			/* rxRefreshRate ~= 10000 */
			rcInterpolationFactor = rxRefreshRate / targetPidLooptime + 1;
//			printf("rcInterpolationFactor: %d\r\n", rcInterpolationFactor);		// rcInterpolationFactor = rxRefreshRate / targetPidLooptime + 1 = 10000 / 500 + 1 ~= 20 or 21
			
			for (int channel = ROLL; channel < interpolationChannels; channel++) {
//				if (channel == ROLL) {
//					printf("rcCommand[ROLL]: %d\r\n", rcCommand[channel]);
//				} else if (channel == PITCH) {
//					printf("rcCommand[PITCH]: %d\r\n", rcCommand[channel]);
//				}
				deltaRC[channel] = rcCommand[channel] - (lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor);
				lastCommand[channel] = rcCommand[channel];
			}
			
			factor = rcInterpolationFactor - 1;
		} else {
			factor--;		// Whenever no data received from the transmitter, the factor is decreased by 1.
		}
		
		/* Interpolate steps of rcCommand */
		if (factor > 0) {
			for (int channel = ROLL; channel < interpolationChannels; channel++) {
				rcCommand[channel] = lastCommand[channel] - deltaRC[channel] * factor / rcInterpolationFactor;
				/* Throttle channel does not require rate calculation */
				readyToCalculateRateAxisCnt = MAX(channel, FD_YAW);
				readyToCalculateRate = true;
			}
		} else {
			factor = 0;
		}
	} else {
		/* Reset factor in case of level modes flip flopping */
		factor = 0;
	}
	
	if (readyToCalculateRate || isRXDataNew) {
		if (isRXDataNew) {
			readyToCalculateRateAxisCnt = FD_YAW;
		}
		
		for (int axis = 0; axis <= readyToCalculateRateAxisCnt; axis++) {
			calculateSetpointRate(axis);
		}
		
		/* Scaling of Angle rate to camera angle (i.e. Mixing Roll and Yaw) */
		if (RxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !FLIGHT_MODE(HEADFREE_MODE)) {
			scaleRcCommandToFpvCamAngle();
		}
		
		isRXDataNew = false;
	}
}
