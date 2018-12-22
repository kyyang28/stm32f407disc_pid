#ifndef __COMMON_H
#define __COMMON_H

#include "stm32f4xx.h"
#include "rx.h"

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
#define DEFAULT_AUX_CHANNEL_COUNT					MAX_AUX_CHANNEL_COUNT		// DEFAULT_AUX_CHANNEL_COUNT = 18 - 4 = 14
#else
#define DEFAULT_AUX_CHANNEL_COUNT					6
#endif

#endif	// __COMMON_H
