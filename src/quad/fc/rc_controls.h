#ifndef __RC_CONTROLS_H
#define __RC_CONTROLS_H

#include <stdint.h>
#include <stdbool.h>
#include "motors.h"
#include "pid.h"

#define MAX_MODE_ACTIVATION_CONDITION_COUNT 				20

typedef enum {
	BOXARM = 0,
	BOXANGLE,
	BOXHORIZON,
	BOXBARO,
	BOXANTIGRAVITY,
	BOXMAG,
	BOXHEADFREE,
	BOXHEADADJ,
	BOXCAMSTAB,
	BOXCAMTRIG,
	BOXGPSHOME,
	BOXGPSHOLD,
	BOXPASSTHRU,
	BOXBEEPERON,
	BOXLEDMAX,
	BOXLEDLOW,
	BOXLLIGHTS,
	BOXCALIB,
	BOXGOV,
	BOXOSD,
	BOXTELEMETRY,
	BOXGTUNE,
	BOXSONAR,
	BOXSERVO1,
	BOXSERVO2,
	BOXSERVO3,
	BOXBLACKBOX,
	BOXFAILSAFE,
	BOXAIRMODE,
	BOX3DDISABLESWITCH,
	BOXFPVANGLEMIX,
	CHECKBOX_ITEM_COUNT
}boxId_e;

typedef enum rc_alias {
	ROLL = 0,
	PITCH,			// Channel 1
	YAW,			// Channel 2
	THROTTLE,		// Channel 3
	AUX1,			// Channel 4
	AUX2,			// Channel 5
	AUX3,
	AUX4,
	AUX5,
	AUX6,
	AUX7,
	AUX8
}rc_alias_e;

typedef enum {
	RC_SMOOTHING_OFF = 0,
	RC_SMOOTHING_DEFAULT,
	RC_SMOOTHING_AUTO,
	RC_SMOOTHING_MANUAL
}rcSmoothing_t;

#define CHANNEL_RANGE_MIN								900
#define CHANNEL_RANGE_MAX								2100

#define MODE_STEP_TO_CHANNEL_VALUE(step)				(CHANNEL_RANGE_MIN + 25 * step)

/*
 * steps are 25 apart
 * a value of 0 corresponds to a channel value of 900 or less
 * a value of 48 corresponds to a channel value of 2100 or more
 * 48 steps between 900 and 2100	( (2100 - 900) / 25 = 48 steps )
 */
typedef struct channelRange_s {
	uint8_t startStep;
	uint8_t endStep;
}channelRange_t;

typedef struct modeActivationCondition_s {
	boxId_e modeId;
	uint8_t auxChannelIndex;
	channelRange_t range;
}modeActivationCondition_t;

typedef struct modeActivationProfile_s {
	modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
}modeActivationProfile_t;

#define IS_RANGE_USABLE(range)			((range)->startStep < (range)->endStep)

typedef struct controlRateConfig_s {
	uint8_t rcRate8;
	uint8_t rcYawRate8;
	uint8_t rcExpo8;
	uint8_t thrMid8;
	uint8_t thrExpo8;
	uint8_t rates[3];							// rc rates, initial values 70
	uint8_t dynThrPID;							// TPA percentage value, default 10 which is 10 / 100 = 0.10 in BF PID section
	uint8_t rcYawExpo8;
	uint16_t tpa_breakpoint;					// Breakpoint where TPA is activated
}controlRateConfig_t;

extern int16_t rcCommand[4];

typedef struct rcControlsConfig_s {
	uint8_t deadband;					// introduce a deadband around the stick centre for pitch and roll axis. Must be greater than zero.
	uint8_t yaw_deadband;				// introduce a deadband around the stick centre for yaw axis. Must be greater than zero.
	uint8_t alt_hold_deadband;			// defines the neutral zone of throttle stick during altitude hold, default setting is +/-40.
	uint8_t alt_hold_fast_change;		// when disabled, turn off the althold when throttle stick is out of deadband defined with alt_hold_deadband; when enabled, altitude changes slowly proportional to stick movement. 
	int8_t yaw_control_direction;		// change control direction of yaw (inverted, normal)
}rcControlsConfig_t;

typedef struct armingConfig_s {
	uint8_t gyro_cal_on_first_arm;		// allow disarm/arm on throttle down + roll left/right
	uint8_t disarm_kill_switch;			// allow disarm via AUX switch regardless of throttle value
	uint8_t auto_disarm_delay;			// allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0.
}armingConfig_t;

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, motorConfig_t *motorConfigToUse, pidProfile_t *pidProfileToUse);
void processRcStickPositions(void);

#endif	// __RC_CONTROLS_H
