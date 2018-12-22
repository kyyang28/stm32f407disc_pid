#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "acceleration.h"

#define MAX_PID_PROCESS_DENOM			16
#define PIDSUM_LIMIT					0.5f
#define PIDSUM_LIMIT_YAW				0.5f

typedef enum {
	PIDROLL,
	PIDPITCH,
	PIDYAW,
	PIDALT,
	PIDPOS,
	PIDPOSR,
	PIDNAVR,
	PIDLEVEL,
	PIDMAG,
	PIDVEL,
	PID_ITEM_COUNT
}pidIndex_e;

typedef enum {
	PID_STABILISATION_OFF = 0,
	PID_STABILISATION_ON
}pidStabilisationState_e;

typedef struct pidProfile_s {
	uint8_t P8[PID_ITEM_COUNT];
	uint8_t I8[PID_ITEM_COUNT];
	uint8_t D8[PID_ITEM_COUNT];
	
	uint8_t dterm_filter_type;						// Filter selection for dterm
	uint16_t dterm_lpf_hz;							// Delta filter in hz
	uint16_t yaw_lpf_hz;							// Additional yaw filter when yaw axis too noisy
	uint16_t dterm_notch_hz;						// Biquad dterm notch hz
	uint16_t dterm_notch_cutoff;					// Biquad dterm notch low cutoff
	uint8_t itermWindupPointPercent;				// Experimental ITerm windup threshold, percent motor saturation
	float pidSumLimit;
	float pidSumLimitYaw;
	uint8_t dterm_average_count;					// Configurable delta count for dterm
	uint8_t vbatPidCompensation;					// Scale PIDsum to battery voltage
	uint8_t pidAtMinThrottle;						// Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active
	uint8_t levelAngleLimit;						// Max angle in degrees in level mode
	uint8_t levelSensitivity;						// Angle mode sensitivity reflected in degrees assuming user using full stick
	
	/* PID controller parameters */
	uint16_t itermThrottleThreshold;				// Max allowed throttle delta before iterm accelerated in ms
	float itermAcceleratorGain;						// Iterm Accelerator Gain when itermThrottleThreshold is hit
	uint8_t setpointRelaxRatio;						// Setpoint weight relaxation effect
	uint8_t dtermSetpointWeight;					// Setpoint weight for Dterm (0 = measurement, 1 = full error, 1 > aggressive derivative)
	float yawRateAccelLimit;						// Yaw accel limiter for deg/sec/ms
	float rateAccelLimit;							// Accel limiter roll/pitch deg/sec/ms
}pidProfile_t;

typedef struct pidConfig_s {
    uint8_t pid_process_denom;                      // Processing denominator for PID controller vs gyro sampling frequency
}pidConfig_t;

void pidController(const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim);

#endif	// __PID_H
