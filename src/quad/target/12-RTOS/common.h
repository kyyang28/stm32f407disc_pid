#ifndef __COMMON_H
#define __COMMON_H

#include "stm32f4xx.h"
#include "rx.h"
#include "debug.h"

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
#define DEFAULT_AUX_CHANNEL_COUNT					MAX_AUX_CHANNEL_COUNT		// DEFAULT_AUX_CHANNEL_COUNT = 18 - 4 = 14
#else
#define DEFAULT_AUX_CHANNEL_COUNT					6
#endif

#define DEBUG_MODE			DEBUG_NONE
//#define SCHEDULER_DEBUG		// define this to use scheduler debug[] values. Undefined by default for performance reasons

#if defined(STM32F4)
#define TASK_GYROPID_DESIRED_PERIOD                 125                 // in usec
#define SCHEDULER_DELAY_LIMIT                       10
#else
#define TASK_GYROPID_DESIRED_PERIOD                 1000                // in usec
#define SCHEDULER_DELAY_LIMIT                       100
#endif

#endif	// __COMMON_H
