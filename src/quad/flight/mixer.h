#ifndef __MIXER_H
#define __MIXER_H

#include <stdint.h>
#include "pid.h"
#include "time.h"

//#define MAX_SUPPORTED_MOTORS		14		// due to the MAX TIMER COUNT = 14, change MAX SUPPORTED MOTORS to be 14 as well for ResetMotorConfig() function
#define MAX_SUPPORTED_MOTORS		12

#define QUAD_MOTOR_COUNT			4

typedef enum mixerMode {
    MIXER_TRI = 1,
    MIXER_QUADP = 2,
    MIXER_QUADX = 3,
    MIXER_BICOPTER = 4,
    MIXER_GIMBAL = 5,
    MIXER_Y6 = 6,
    MIXER_HEX6 = 7,
    MIXER_FLYING_WING = 8,
    MIXER_Y4 = 9,
    MIXER_HEX6X = 10,
    MIXER_OCTOX8 = 11,
    MIXER_OCTOFLATP = 12,
    MIXER_OCTOFLATX = 13,
    MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    MIXER_HELI_120_CCPM = 15,
    MIXER_HELI_90_DEG = 16,
    MIXER_VTAIL4 = 17,
    MIXER_HEX6H = 18,
    MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
    MIXER_DUALCOPTER = 20,
    MIXER_SINGLECOPTER = 21,
    MIXER_ATAIL4 = 22,
    MIXER_CUSTOM = 23,
    MIXER_CUSTOM_AIRPLANE = 24,
    MIXER_CUSTOM_TRI = 25,
    MIXER_QUADX_1234 = 26
}mixerMode_e;

/* Custom mixer data per motor */
typedef struct motorMixer_s {
	float throttle;
	float roll;
	float pitch;
	float yaw;
}motorMixer_t;

/* Custom mixer configuration */
typedef struct mixer_s {
	uint8_t motorCount;
	uint8_t useServo;
	const motorMixer_t *motor;
}mixer_t;

typedef struct mixerConfig_s {
	uint8_t mixerMode;
	int8_t yaw_motor_direction;
}mixerConfig_t;

extern int16_t motor[MAX_SUPPORTED_MOTORS];
extern int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];

struct motorConfig_s;			// cheat on compiler
struct rxConfig_s;				// cheat on compiler

uint8_t getMotorCount(void);
float getMotorMixRange(void);
void writeMotors(void);
void mixTable(struct pidProfile_s *pidProfile);
void mixerInit(mixerMode_e mixerMode, motorMixer_t *initialCustomMixers);
void mixerConfigurationOutput(void);
void mixerUseConfigs(struct motorConfig_s *motorConfigToUse, mixerConfig_t *mixerConfigToUse, struct rxConfig_s *rxConfigToUse);

#endif	// __MIXER_H
