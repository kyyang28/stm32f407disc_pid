#ifndef __RC_CONTROLS_H
#define __RC_CONTROLS_H

#include <stdint.h>
#include <stdbool.h>
#include "motors.h"
#include "pid.h"
#include "maths.h"
#include "rx.h"

#define MAX_MODE_ACTIVATION_CONDITION_COUNT 				20

#define ROL_LO						(1 << (2 * ROLL))			// 1 << 0
#define ROL_CE						(3 << (2 * ROLL))			// 3 << 0
#define ROL_HI						(2 << (2 * ROLL))			// 2 << 0
#define PIT_LO						(1 << (2 * PITCH))			// 1 << 2
#define PIT_CE						(3 << (2 * PITCH))			// 3 << 2
#define PIT_HI						(2 << (2 * PITCH))			// 2 << 2
#define YAW_LO						(1 << (2 * YAW))			// 1 << 4
#define YAW_CE						(3 << (2 * YAW))			// 3 << 4
#define YAW_HI						(2 << (2 * YAW))			// 2 << 4
#define THR_LO						(1 << (2 * THROTTLE))		// 1 << 6
#define THR_CE						(3 << (2 * THROTTLE))		// 3 << 6
#define THR_HI						(2 << (2 * THROTTLE))		// 2 << 6

typedef enum {
	BOXARM = 0,			// 0
	BOXANGLE,			// 1
	BOXHORIZON,			// 2
	BOXBARO,			// 3
	BOXANTIGRAVITY,		// 4
	BOXMAG,				// 5
	BOXHEADFREE,		// 6
	BOXHEADADJ,			// 7
	BOXCAMSTAB,			// 8
	BOXCAMTRIG,			// 9
	BOXGPSHOME,			// 10
	BOXGPSHOLD,			// 11
	BOXPASSTHRU,		// 12
	BOXBEEPERON,		// 13
	BOXLEDMAX,			// 14
	BOXLEDLOW,			// 15
	BOXLLIGHTS,			// 16
	BOXCALIB,			// 17
	BOXGOV,				// 18
	BOXOSD,				// 19
	BOXTELEMETRY,		// 20
	BOXGTUNE,			// 21
	BOXSONAR,			// 22
	BOXSERVO1,			// 23
	BOXSERVO2,			// 24
	BOXSERVO3,			// 25
	BOXBLACKBOX,		// 26
	BOXFAILSAFE,		// 27
	BOXAIRMODE,			// 28
	BOX3DDISABLESWITCH,	// 29
	BOXFPVANGLEMIX,		// 30
	CHECKBOX_ITEM_COUNT	// 31
}boxId_e;

extern uint32_t rcModeActivationMask;
#define IS_RC_MODE_ACTIVE(modeId)	((1 << (modeId)) & rcModeActivationMask)
#define ACTIVATE_RC_MODE(modeId)	(rcModeActivationMask |= (1 << modeId))

typedef enum rc_alias {
	ROLL = 0,		// Channel 0
	PITCH,			// Channel 1
	YAW,			// Channel 2
	THROTTLE,		// Channel 3
	AUX1,			// Channel 4
	AUX2,			// Channel 5
	AUX3,			// Channel 6
	AUX4,			// Channel 7
	AUX5,			// Channel 8
	AUX6,			// Channel 9
	AUX7,			// Channel 10
	AUX8			// Channel 11
}rc_alias_e;

typedef enum {
	RC_SMOOTHING_OFF = 0,			// 0
	RC_SMOOTHING_DEFAULT,			// 1
	RC_SMOOTHING_AUTO,				// 2
	RC_SMOOTHING_MANUAL				// 3
}rcSmoothing_t;

#define CHANNEL_RANGE_MIN								900
#define CHANNEL_RANGE_MAX								2100

#define MIN_MODE_RANGE_STEP								0
#define MAX_MODE_RANGE_STEP								((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / 25)		// (2100 - 900) / 25 = 48

#define MODE_STEP_TO_CHANNEL_VALUE(step)				(CHANNEL_RANGE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_STEP(channelValue)				((constrain(channelValue, CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX) - CHANNEL_RANGE_MIN) / 25)

/*
 * steps are 25 apart, actually it should be 24 apart
 * a value of 0 corresponds to a channel value of 900 or less
 * a value of 48 corresponds to a channel value of 2100 or more
 * 48 steps between 900 and 2100	( (2100 - 900) / 25 = 48 steps )
 *
 * In MODE section to setup the ARM, Flight modes, airmode and so on using AUX1, AUX2, AUX3, etc
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

typedef enum {
	THROTTLE_LOW = 0,					// 0
	THROTTLE_HIGH						// 1
}throttleStatus_e;

bool isAntiGravityModeActive(void);
bool isAirModeActive(void);
bool isUsingSticksForArming(void);
void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, motorConfig_t *motorConfigToUse, pidProfile_t *pidProfileToUse);
void processRcStickPositions(struct rxConfig_s *rxConfig, throttleStatus_e throttleStatus, bool disarm_kill_switch);

throttleStatus_e calculateThrottleStatus(struct rxConfig_s *rxConfig);

#endif	// __RC_CONTROLS_H
