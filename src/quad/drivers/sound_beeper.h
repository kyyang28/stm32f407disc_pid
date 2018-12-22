#ifndef __SOUND_BEEPER_H
#define __SOUND_BEEPER_H

#include "io.h"

typedef struct beeperConfig_s {
	ioTag_t ioTag;
	uint8_t isInverted;
	uint8_t isOpenDrain;
}beeperConfig_t;

void beeperInit(const beeperConfig_t *beeperConfig);
void systemBeep(bool onoff);
void systemBeeperToggle(void);

#ifdef BEEPER
#define BEEP_TOGGLE					systemBeeperToggle()
#define BEEP_OFF					systemBeep(false)
#define BEEP_ON						systemBeep(true)
#else
#define BEEP_TOGGLE					do {} while(0)
#define BEEP_OFF					do {} while(0)
#define BEEP_ON						do {} while(0)
#endif

#endif	// __SOUND_BEEPER_H
