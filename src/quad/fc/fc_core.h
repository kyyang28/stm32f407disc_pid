#ifndef __FC_CORE_H
#define __FC_CORE_H

#include <stdbool.h>
#include "time.h"

extern bool isRXDataNew;

void updateLEDs(void);
void mwArm(void);

void processRx(timeUs_t currentTimeUs);
void taskMainPidLoop(timeUs_t currentTimeUs);

#endif	// __FC_CORE_H
