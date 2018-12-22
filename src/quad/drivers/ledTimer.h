#ifndef __LEDTIMER_H
#define __LEDTIMER_H

#include "io.h"

#define LED_NUMBER			4

//#define LED3_NUM			0
//#define LED4_NUM			1
//#define LED5_NUM			2
//#define LED6_NUM			3

typedef struct {
	ioTag_t ioTags[LED_NUMBER];	// ledTags[0]: LED3, ledTags[1]: LED4, ledTags[2]: LED5, ledTags[3]: LED6
}LedTimerConfig_t;

#endif	// __LEDTIMER_H
