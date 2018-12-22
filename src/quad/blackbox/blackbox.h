#ifndef __BLACKBOX_H
#define __BLACKBOX_H

#include "time.h"

typedef struct blackboxConfig_s {
	uint8_t rate_num;
	uint8_t rate_denom;
	uint8_t device;
	uint8_t on_motor_test;
}blackboxConfig_t;

typedef enum {
	BLACKBOX_SERIAL = 0,
	BLACKBOX_SPIFLASH,
	BLACKBOX_SDCARD
}blackBoxDevice_e;

void handleBlackbox(timeUs_t currentTimeUs);

void initBlackbox(void);
void startBlackbox(void);

void validateBlackboxConfig(void);

#endif	// __BLACKBOX_H
