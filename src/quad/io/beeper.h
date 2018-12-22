#ifndef __BEEPER_H
#define __BEEPER_H

#include <stdint.h>
#include "time.h"

/* IMPORTANT: these are in priority order, 0 = Highest */
typedef enum {
	BEEPER_SILENCE = 0,											// Silence, see beeperSilence()
	BEEPER_GYRO_CALIBRATED,
	/* Add other BEEPER_XXX here */
	BEEPER_SYSTEM_INIT,											// Initialisation beeps when board is powered on
	
	BEEPER_ALL,													// Turn ON or OFF all beeper conditions
	BEEPER_PREFERENCE											// Save preferred beeper configuration
}beeperMode_e;

void beeper(beeperMode_e mode);
void beeperSilence(void);
void beeperUpdate(timeUs_t currentTimeUs);
void beeperConfirmationBeeps(uint8_t beepCount);

#endif	// __BEEPER_H
