#ifndef __RUNTIME_CONFIG_H
#define __RUNTIME_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

extern uint8_t armingFlags;

typedef enum {
	OK_TO_ARM			= (1 << 0),
	PREVENT_ARMING		= (1 << 1),
	ARMED				= (1 << 2),
	WAS_EVER_ARMED		= (1 << 3)
}armingFlag_e;

#define DISABLE_ARMING_FLAG(mask)			(armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask)			(armingFlags |= (mask))
#define CHECK_ARMING_FLAG(mask)				(armingFlags & (mask))

bool sensors(uint32_t mask);
void sensorSet(uint32_t mask);
void sensorClear(uint32_t mask);
uint32_t sensorsMask(void);

#endif	// __RUNTIME_CONFIG_H
