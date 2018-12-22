
#include "beeper.h"
#include "target.h"
#include "utils.h"
#include "config.h"

#ifdef BEEPER
void beeper(beeperMode_e mode)
{
	
}

void beeperSilence(void)
{
	
}

void beeperUpdate(timeUs_t currentTimeUs)
{
	
}

void beeperConfirmationBeeps(uint8_t beepCount)
{
	
}
#else
void beeper(beeperMode_e mode) { UNUSED(mode); }
void beeperSilence(void) {}
void beeperUpdate(timeUs_t currentTimeUs) { UNUSED(currentTimeUs); }
void beeperConfirmationBeeps(uint8_t beepCount) { UNUSED(beepCount); }
#endif
