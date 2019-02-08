#ifndef __RUNTIME_CONFIG_H
#define __RUNTIME_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

extern uint8_t armingFlags;
extern uint16_t flightModeFlags;
extern uint8_t stateFlags;

typedef enum {
	OK_TO_ARM			= (1 << 0),
	PREVENT_ARMING		= (1 << 1),
	ARMED				= (1 << 2),
	WAS_EVER_ARMED		= (1 << 3)
}armingFlag_e;

typedef enum {
	ANGLE_MODE			= (1 << 0),
	HORIZON_MODE		= (1 << 1),
	MAG_MODE			= (1 << 2),
	BARO_MODE			= (1 << 3),
	GPS_HOME_MODE		= (1 << 4),
	GPS_HOLD_MODE		= (1 << 5),
	HEADFREE_MODE		= (1 << 6),
	UNUSED_MODE			= (1 << 7),		// old autotune
	PASSTHRU_MODE		= (1 << 8),
	SONAR_MODE			= (1 << 9),
	FAILSAFE_MODE		= (1 << 10)
}flightModeFlags_e;

#define DISABLE_ARMING_FLAG(mask)			(armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask)			(armingFlags |= (mask))
#define CHECK_ARMING_FLAG(mask)				(armingFlags & (mask))

#define ENABLE_FLIGHT_MODE(mask)			enableFlightMode(mask)
#define DISABLE_FLIGHT_MODE(mask)			disableFlightMode(mask)
#define FLIGHT_MODE(mask)					(flightModeFlags & (mask))

typedef enum {
	GPS_FIX_HOME		= (1 << 0),
	GPS_FIX				= (1 << 1),
	CALIBRATE_MAG		= (1 << 2),
	SMALL_ANGLE			= (1 << 3),
	FIXED_WING			= (1 << 4)
}stateFlags_t;

#define ENABLE_STATE(mask)					(stateFlags &= ~(mask))
#define DISABLE_STATE(mask)					(stateFlags |= (mask))
#define CHECK_STATE_FLAG(mask)				(stateFlags & (mask))

bool sensors(uint32_t mask);
void sensorSet(uint32_t mask);
void sensorClear(uint32_t mask);
uint32_t sensorsMask(void);

uint16_t enableFlightMode(flightModeFlags_e mask);
uint16_t disableFlightMode(flightModeFlags_e mask);

#endif	// __RUNTIME_CONFIG_H
