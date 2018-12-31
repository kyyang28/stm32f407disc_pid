
#include <stdio.h>          	// just for debugging purposes

#include "pid.h"
#include "fc_rc.h"
#include "mixer.h"
#include "filter.h"
#include "maths.h"				// MIN, MAX
#include "runtime_config.h"

#include "configMaster.h"   	// just for testing purposes

//#define TESTING_TPA

//#if defined(TESTING_TPA)
//#include "rx.h"
//extern int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
//#endif

uint32_t targetPidLooptime;

static float dT;

float axisPID_P[3], axisPID_I[3], axisPID_D[3];

static bool pidStabilisationEnabled;

/* Declarations of PID related filters */
static filterApplyFnPtr dtermNotchFilterApplyFn;
static void *dtermFilterNotch[2];					// 2 means we are handling TWO axis which are ROLL and PITCH
static filterApplyFnPtr dtermLpfApplyFn;
static void *dtermFilterLpf[2];
static filterApplyFnPtr ptermYawFilterApplyFn;
static void *ptermYawFilter;
/* Declarations of PID related filters */

/* Declarations of PID related configurations */
static float Kp[3], Ki[3], Kd[3], maxVelocity[3];
static float relaxFactor;
static float dtermSetpointWeight;
static float levelGain, horizonGain, horizonTransition, ITermWindupPoint, ITermWindupPointInv;
static float itermAccelerator = 1.0f;
/* Declarations of PID related configurations */

void pidSetTargetLooptime(uint32_t pidLooptime)
{
	targetPidLooptime = pidLooptime;
	
	/* set dt in seconds (targetPidLooptime (in microseconds) to seconds)
	 *
	 * For example, targetPidLooptime = 500.
	 * dt = targetPidLooptime * 0.000001 = 500 * 0.000001f = 0.0005 (for F210 racing quad)
     *
 	 * For example, targetPidLooptime = 4000.
	 * dt = targetPidLooptime * 0.000001 = 4000 * 0.000001f = 0.004 (for F450 normal quad)
	 */
	dT = targetPidLooptime * 0.000001f;
}

void pidSetItermAccelerator(float newItermAccelerator)
{
	itermAccelerator = newItermAccelerator;
}

void pidInitFilters(const pidProfile_t *pidProfile)
{
	static biquadFilter_t biquadFilterNotch[2];
	static pt1Filter_t pt1Filter[2];
	static biquadFilter_t biquadFilter[2];
	static pt1Filter_t pt1FilterYaw;
//	static firFilterDenoise_t denoisingFilter[2];		// TODO: FIR filter might be implemented later
	
	/* 1. PID Nyquist frequency, no rounding needed */
	uint32_t pidNyquistFrequency = (1.0f / dT) / 2;		// pidNyquistFrequency unit in hz, dT = (1.0f / 0.0005) / 2 = 2000 / 2 = 1000
	
//	BUILD_BUG_ON(FD_YAW != 2); // only setting up Dterm filters on roll and pitch axes, so ensure yaw axis is 2
	
	/* 2. dterm notch filter initialisation */
	if (pidProfile->dterm_notch_hz == 0 || pidProfile->dterm_notch_hz > pidNyquistFrequency) {
		dtermNotchFilterApplyFn = nullFilterApply;
	} else {
		dtermNotchFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
		const float notchQ = filterGetNotchQ(pidProfile->dterm_notch_hz, pidProfile->dterm_notch_cutoff);
		
		/* FD_ROLL = 0, FD_PITCH = 1 */
		for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
			dtermFilterNotch[axis] = &biquadFilterNotch[axis];
			
			/* void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType)
			 *
			 * targetPidLooptime = gyro.targetLooptime * PidConfig()->pid_process_denom = 125 (us) * 4 = 500 (us)
			 */
			biquadFilterInit(dtermFilterNotch[axis], pidProfile->dterm_notch_hz, targetPidLooptime, notchQ, FILTER_NOTCH);
		}
	}
	
	/* 3. dterm lowpass filter initialisation */
	if (pidProfile->dterm_lpf_hz == 0 || pidProfile->dterm_lpf_hz > pidNyquistFrequency) {
		dtermLpfApplyFn = nullFilterApply;
	} else {
		switch (pidProfile->dterm_filter_type) {
			case FILTER_PT1:
				dtermLpfApplyFn = (filterApplyFnPtr)pt1FilterApply;
				for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
					dtermFilterLpf[axis] = &pt1Filter[axis];
					
					/* void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT)
					 *
					 * pidProfile->dterm_lpf_hz = 100 (cutoff frequency)
					 * dT = 500 * 0.000001f = 0.0005
					 */
					pt1FilterInit(dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, dT);
				}
				break;
			
			case FILTER_BIQUAD:
				dtermLpfApplyFn = (filterApplyFnPtr)biquadFilterApply;
				for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
					dtermFilterLpf[axis] = &biquadFilter[axis];
					
					/*
					 * void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
					 * 
					 * pidProfile->dterm_lpf_hz = 100 (cutoff frequency)
					 * targetPidLooptime = gyro.targetLooptime * PidConfig()->pid_process_denom = 125 (us) * 4 = 500 (us)
					 */
					biquadFilterInitLPF(dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, targetPidLooptime);
				}
				break;
			
			case FILTER_FIR:
				/* TODO: To be implemented */
				break;
			
			default:
				dtermLpfApplyFn = nullFilterApply;
				break;
		}
	}
	
	/* 4. yaw lowpass filter initialisation */
	if (pidProfile->yaw_lpf_hz == 0 || pidProfile->yaw_lpf_hz > pidNyquistFrequency) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		ptermYawFilterApplyFn = nullFilterApply;
	} else {
		ptermYawFilterApplyFn = (filterApplyFnPtr)pt1FilterApply;
		ptermYawFilter = &pt1FilterYaw;
		pt1FilterInit(ptermYawFilter, pidProfile->yaw_lpf_hz, dT);
	}
}

void pidInitConfig(const pidProfile_t *pidProfile)
{
	/*
	 * PTERM_SCALE = 0.032029f
	 * ITERM_SCALE = 0.244381f
	 * DTERM_SCALE = 0.000529f
	 * 
	 * pidProfile->P8[FD_ROLL] = pidProfile->P8[0] = 44
	 * pidProfile->I8[FD_ROLL] = pidProfile->I8[0] = 40
	 * pidProfile->D8[FD_ROLL] = pidProfile->D8[0] = 30
	 * pidProfile->P8[FD_PITCH] = pidProfile->P8[1] = 58;
	 * pidProfile->I8[FD_PITCH] = pidProfile->I8[1] = 50;
	 * pidProfile->D8[FD_PITCH] = pidProfile->D8[1] = 35;
	 * pidProfile->P8[FD_YAW] = pidProfile->P8[2] = 70;
	 * pidProfile->I8[FD_YAW] = pidProfile->I8[2] = 45;
	 * pidProfile->D8[FD_YAW] = pidProfile->D8[2] = 20;
	 *
	 * Kp[FD_ROLL] = Kp[0] = PTERM_SCALE * pidProfile->P8[FD_ROLL] = 0.032029f * 44 = 1.409276
	 * Ki[FD_ROLL] = Ki[0] = ITERM_SCALE * pidProfile->I8[FD_ROLL] = 0.244381f * 40 = 9.77524
	 * Kd[FD_ROLL] = Kd[0] = DTERM_SCALE * pidProfile->D8[FD_ROLL] = 0.000529f * 30 = 0.01587
	 *
	 * Kp[FD_PITCH] = Kp[1] = PTERM_SCALE * pidProfile->P8[FD_PITCH] = 0.032029f * 58 = 1.857682
	 * Ki[FD_PITCH] = Ki[1] = ITERM_SCALE * pidProfile->I8[FD_PITCH] = 0.244381f * 50 = 12.21905
	 * Kd[FD_PITCH] = Kd[1] = DTERM_SCALE * pidProfile->D8[FD_PITCH] = 0.000529f * 35 = 0.018515
	 *
	 * Kp[FD_YAW] = Kp[2] = PTERM_SCALE * pidProfile->P8[FD_YAW] = 0.032029f * 70 = 2.24203
	 * Ki[FD_YAW] = Ki[2] = ITERM_SCALE * pidProfile->I8[FD_YAW] = 0.244381f * 45 = 10.997145
	 * Kd[FD_YAW] = Kd[2] = DTERM_SCALE * pidProfile->D8[FD_YAW] = 0.000529f * 20 = 0.01058
 	 */
	for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
		Kp[axis] = PTERM_SCALE * pidProfile->P8[axis];
		Ki[axis] = ITERM_SCALE * pidProfile->I8[axis];
		Kd[axis] = DTERM_SCALE * pidProfile->D8[axis];
//		printf("Kp[%d]: %f\tKi[%d]: %f\tKd[%d]: %f\r\n", axis, Kp[axis], axis, Ki[axis], axis, Kd[axis]);
	}
	
	/* pidProfile->dtermSetpointWeight = 60
	 * 
	 * dtermSetpointWeight = pidProfile->dtermSetpointWeight / 127.0f = 60 / 127.0f = 0.47244094488188976377952755905512
	 */
	dtermSetpointWeight = pidProfile->dtermSetpointWeight / 127.0f;
//	printf("dtermSetpointWeight: %f\r\n", dtermSetpointWeight);				// 0.472441
	
	/* pidProfile->setpointRelaxRatio = 100
	 * 
	 * relaxFactor = (1.0f / (pidProfile->setpointRelaxRatio / 100.0f)) = 1.0f / (100 / 100.0f) = 1.0
	 */	
	relaxFactor = 1.0f / (pidProfile->setpointRelaxRatio / 100.0f);
//	printf("relaxFactor: %f\r\n", relaxFactor);								// 1.000000
		
	/* pidProfile->P8[PIDLEVEL] = pidProfile->P8[7] = 50
	 * 
	 * levelGain = pidProfile->P8[PIDLEVEL] / 10.0f = 50 / 10.0f = 5.0
	 */	
	levelGain = pidProfile->P8[PIDLEVEL] / 10.0f;
//	printf("levelGain: %f\r\n", levelGain);					// 5.000000
	
	/* pidProfile->I8[PIDLEVEL] = pidProfile->I8[7] = 50
	 * 
	 * horizonGain = pidProfile->I8[PIDLEVEL] / 10.0f = 50 / 10.0f = 5.0
	 */	
	horizonGain = pidProfile->I8[PIDLEVEL] / 10.0f;
//	printf("horizonGain: %f\r\n", horizonGain);				// 5.000000
	
	/* pidProfile->D8[PIDLEVEL] = pidProfile->D8[7] = 100
	 * 
	 * horizonTransition = 100.0f / pidProfile->D8[PIDLEVEL] = 100.0f / 100 = 1.0
	 */	
	horizonTransition = 100.0f / pidProfile->D8[PIDLEVEL];
//	printf("horizonTransition: %f\r\n", horizonTransition);	// 1.000000
	
	/*
	 * pidProfile->rateAccelLimit = 0.0f
	 *
	 * maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = 0.0f * 1000 * dT = 0.0
	 */
	maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 1000 * dT;
//	printf("maxVelocity[FD_ROLL]: %f\r\n", maxVelocity[FD_ROLL]);	// 0.000000
//	printf("maxVelocity[FD_PITCH]: %f\r\n", maxVelocity[FD_PITCH]);	// 0.000000
	
	/*
	 * pidProfile->yawRateAccelLimit = 10.0f
	 * 
	 * dT = 500 * 0.000001f = 0.0005
	 *
	 * maxVelocity[FD_YAW] = 10.0f * 1000 * dT = 10.0f * 1000 * dT = 10.0f * 1000 * 0.0005 = 5.0 (For F210 racing quad)
	 *
	 * maxVelocity[FD_YAW] = 10.0f * 1000 * dT = 10.0f * 1000 * dT = 10.0f * 1000 * 0.004 = 40.0 (For F450 normal quad)
	 */
	maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 1000 * dT;
//	printf("maxVelocity[FD_YAW]: %f\r\n", maxVelocity[FD_YAW]);		// 5.000000	for F210 quad, 40.0 for F450 quad
	
	/*
	 * pidProfile->itermWindupPointPercent = 50
	 *
	 * ITermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f = 50 / 100.0f = 0.5
	 */	
	ITermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f;
//	printf("ITermWindupPoint: %f\r\n", ITermWindupPoint);			// 0.500000
	
	/* ITermWindupPoint Inverted value
	 *
	 * ITermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f = 50 / 100.0f = 0.5
	 *
	 * ITermWindupPointInv = 1.0f / (1.0f - 0.5f) = 1.0f / 0.5f = 2.0
	 */
	ITermWindupPointInv = 1.0f / (1.0f - ITermWindupPoint);
//	printf("ITermWindupPointInv: %f\r\n", ITermWindupPointInv);		// 2.000000
}

void pidResetErrorGyroState(void)
{
	for (int axis = 0; axis < 3; axis++) {
		axisPID_I[axis] = 0.0f;
	}
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
	pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

static float accelerationLimit(int axis, float currentPidSetpoint)
{
	static float previousSetpoint[3];
	const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];
//	printf("curr[%d]: %f\r\n", axis, currentPidSetpoint);
//	printf("prev[%d]: %f\r\n", axis, previousSetpoint[axis]);
//	printf("vel[%d]: %f\r\n", axis, currentVelocity);
//	printf("maxVel[%d]: %f\r\n", axis, maxVelocity[axis]);
	
	/* 
	 * axis = YAW
	 * maxVelocity[YAW] = pidProfile->yawRateAccelLimit * 1000 * dT = 10.0 * 1000 * 0.004 = 40.0f
	 * maxVelocity[ROLL] = maxVelocity[PITCH] = 0.0f
	 */
	if (ABS(currentVelocity) > maxVelocity[axis]) {
		currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity[axis] : previousSetpoint[axis] - maxVelocity[axis];
//		printf("currentPidSetpoint: %f\r\n", currentPidSetpoint);
	}
	
	previousSetpoint[axis] = currentPidSetpoint;
	
	return currentPidSetpoint;
}

/* 2-DOF PID controller based on MATLAB */
void pidController(const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim)
{
    static float previousRateError[2];
    const float tpaFactor = getThrottlePIDAttenuation();
    const float motorMixRange = getMotorMixRange();

//#if defined(TESTING_TPA)    
//    /* Throttle PID Attenuation (in %): 88%            Throttle value: 1617            TPA threshold: 1350 */
//    printf("Throttle PID Attenuation (in %%): %d%%\tThrottle value: %d\tTPA threshold: %u\r\n", (int)(tpaFactor*100), rcData[THROTTLE], currentProfile->controlRateProfile[0].tpa_breakpoint);
//#endif
    
	/* Dynamic Ki component to gradually scale back integration when above windup point */
    const float dynKi = MIN((1.0f - motorMixRange) * ITermWindupPointInv, 1.0f);
	
	/* The actual PID controller algorithms */
	for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
		
		float currentPidSetpoint = getSetpointRate(axis);
//		printf("setpoint[%d]: %f\r\n", axis, currentPidSetpoint);
		
		if (maxVelocity[axis]) {
			currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
//			printf("currentPidSetpoint: %f\r\n", currentPidSetpoint);
		}
		
		/* YAW control is GYRO-based, direct sticks control is applied to rate PID */
		if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && axis != YAW) {
//			currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);		// Implement pidLevel later
		}
		
		/* process variable from gyro output in deg/sec */
		const float gyroRate = gyro.gyroADCf[axis];
		
//		if (axis == 2)
//			printf("gyroRate[%d]: %f\r\n", axis, gyroRate);
		
		/* +------------------------------ Low-level gyro-based 2DOF PID controller ------------------------------+
		 *
		 * Referenced by 2-DOF PID controller of MATLAB
		 * https://uk.mathworks.com/help/control/ug/two-degree-of-freedom-2-dof-pid-controllers.html
		 * https://uk.mathworks.com/help/control/ug/tune-2-dof-pid-controller-pid-tuner.html
		 *
		 * In essence, using 2-DOF control can improve disturbance rejection without sacrificing as much reference 
		 * tracking performance as 1-DOF control. These effects on system performance depend strongly on the properties 
		 * of your plant and the speed of your controller. For some plants and some control bandwidths, using 2-DOF 
		 * control or changing the design focus has less or no impact on the tuned result.
		 * 
		 * 2-DOF PID controller with optional filter on derivative term.
		 * coefficient b = 1 and only c (setpoint weight on derivative term) can be tuned (amount derivative on measurement or error).
		 */
		
		/* +--------------------------------------------------------------------------------------------------+ */
		/* +-------------------------------------- Calculate error rate --------------------------------------+ */
		/* +--------------------------------------------------------------------------------------------------+ */
		const float errorRate = currentPidSetpoint - gyroRate;			// ref - output i.e. r - y
//		printf("errorRate[%d]: %f\r\n", axis, errorRate);

		/* +--------------------------------------------------------------------------------------------------+ */
		/* +-----------------  Calculate P parameter and add dynamic part based on stick input ---------------+ */
		/* +--------------------------------------------------------------------------------------------------+
		 *
		 * #define PTERM_SCALE						0.032029f
		 * #define DTERM_SCALE						0.000529f
		 *
		 * Kp[FD_ROLL] = Kp[0] = PTERM_SCALE * pidProfile->P8[FD_ROLL] = 0.032029f * 44 = 1.409276
		 * Kp[FD_PITCH] = Kp[1] = PTERM_SCALE * pidProfile->P8[FD_PITCH] = 0.032029f * 58 = 1.857682
		 * Kp[FD_YAW] = Kp[2] = PTERM_SCALE * pidProfile->P8[FD_YAW] = 0.032029f * 70 = 2.24203
		 *
		 * tpaFactor = throttlePIDAttenuation ranges between 0.0f to 1.0f
		 */
//		printf("Kp[%d]: %f\r\n", axis, Kp[axis]);
//		printf("tpaFactor: %f\r\n", tpaFactor);
		axisPID_P[axis] = Kp[axis] * errorRate * tpaFactor;
//		if (axis == 0)
//			printf("P[%d]: %f\r\n", axis, axisPID_P[axis]);
	
		if (axis == FD_YAW) {
//			printf("P1: %f\r\n", axisPID_P[axis]);
			axisPID_P[axis] = ptermYawFilterApplyFn(ptermYawFilter, axisPID_P[axis]);
//			printf("P2: %f\r\n", axisPID_P[axis]);
		}

		/* +--------------------------------------------------------------------------------------------------+ */
		/* +------------------------------------- Calculate I parameter --------------------------------------+ */
		/* +--------------------------------------------------------------------------------------------------+
		 *
		 * #define ITERM_SCALE						0.244381f
		 *
 		 * Ki[FD_ROLL] = Ki[0] = ITERM_SCALE * pidProfile->I8[FD_ROLL] = 0.244381f * 40 = 9.77524
		 * Ki[FD_PITCH] = Ki[1] = ITERM_SCALE * pidProfile->I8[FD_PITCH] = 0.244381f * 50 = 12.21905
		 * Ki[FD_YAW] = Ki[2] = ITERM_SCALE * pidProfile->I8[FD_YAW] = 0.244381f * 45 = 10.997145
		 *
		 * Only increase I-term if motors' outputs are not saturated to prevent I-term windup
		 * 
		 * motorMixRange < 1.0f means motors are not saturated.
		 * motorMixRange >= 1.0f means motors ARE saturated.
		 * 
		 * motorMixRange initial value is 0.0f, which is less than 1.0f, calculate the axisPID_I[axis]
		 *
		 * dT = 0.004 for F450 normal quad
		 */
		if (motorMixRange < 1.0f) {
			axisPID_I[axis] += Ki[axis] * errorRate * dT * dynKi * itermAccelerator;
		}
	}
}
