
#include "runtime_config.h"

uint8_t armingFlags = 0;

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
