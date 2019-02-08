
#include "runtime_config.h"

uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;
uint8_t stateFlags = 0;

static uint32_t enabledSensors = 0;

bool sensors(uint32_t mask)
{
	return enabledSensors & mask;
}

void sensorSet(uint32_t mask)
{
	enabledSensors |= mask;
}

void sensorClear(uint32_t mask)
{
	enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
	return enabledSensors;
}

/*
 * Enable the given flight mode.
 * A beep is sound to indicate the flight mode has been changed.
 *
 * @returns
 *		the new flightModeFlags value
 */
uint16_t enableFlightMode(flightModeFlags_e mask)
{
	uint16_t oldVal = flightModeFlags;
	
	flightModeFlags |= (mask);
	
	if (flightModeFlags != oldVal) {
//		beeperConfirmationBeeps(1);		// beeper delay a certain periods, might be counted inside the whole system delay
	}
	
	return flightModeFlags;
}

/*
 * Disable the given flight mode.
 * A beep is sound to indicate the flight mode has been changed.
 *
 * @returns
 *		the new flightModeFlags value
 */
uint16_t disableFlightMode(flightModeFlags_e mask)
{
	uint16_t oldVal = flightModeFlags;
	
	flightModeFlags &= ~(mask);
	
	if (flightModeFlags != oldVal) {
//		beeperConfirmationBeeps(1);		// beeper delay a certain periods, might be counted inside the whole system delay
	}
	
	return flightModeFlags;
}
