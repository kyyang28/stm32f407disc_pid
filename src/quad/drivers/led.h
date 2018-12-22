#ifndef __LED_H
#define __LED_H

#include <stdbool.h>
#include "io.h"
#include "time.h"

#define LED_NUMBER			4

#define LED3_NUM			0
#define LED4_NUM			1
#define LED5_NUM			2
#define LED6_NUM			3

typedef struct {
	ioTag_t ledTags[LED_NUMBER];	// ledTags[0]: LED3, ledTags[1]: LED4, ledTags[2]: LED5, ledTags[3]: LED6
	uint8_t polarity;
}LedStatusConfig_t;

#ifdef LED3
#define LED3_ON				LedSet(LED3_NUM, true)
#define LED3_OFF			LedSet(LED3_NUM, false)
#define LED3_TOGGLE			LedToggle(LED3_NUM)
#else
#define LED3_ON				do {} while (0)
#define LED3_OFF			do {} while (0)
#define LED3_TOGGLE			do {} while (0)
#endif

#ifdef LED4
#define LED4_ON				LedSet(LED4_NUM, true)
#define LED4_OFF			LedSet(LED4_NUM, false)
#define LED4_TOGGLE			LedToggle(LED4_NUM)
#else
#define LED4_ON				do {} while (0)
#define LED4_OFF			do {} while (0)
#define LED4_TOGGLE			do {} while (0)
#endif

#ifdef LED5
#define LED5_ON				LedSet(LED5_NUM, true)
#define LED5_OFF			LedSet(LED5_NUM, false)
#define LED5_TOGGLE			LedToggle(LED5_NUM)
#else
#define LED5_ON				do {} while (0)
#define LED5_OFF			do {} while (0)
#define LED5_TOGGLE			do {} while (0)
#endif

#ifdef LED6
#define LED6_ON				LedSet(LED6_NUM, true)
#define LED6_OFF			LedSet(LED6_NUM, false)
#define LED6_TOGGLE			LedToggle(LED6_NUM)
#else
#define LED6_ON				do {} while (0)
#define LED6_OFF			do {} while (0)
#define LED6_TOGGLE			do {} while (0)
#endif
	
void LedInit(LedStatusConfig_t *ledStatusConfig);
void LedToggle(int ledNum);
void LedSet(int ledNum, bool ledState);
    
void taskLed3(timeUs_t currentTimeUs);           // task for RTOS    
void taskLed4(timeUs_t currentTimeUs);           // task for RTOS    
void taskLed5(timeUs_t currentTimeUs);           // task for RTOS    
void taskLed6(timeUs_t currentTimeUs);           // task for RTOS    

#endif	// __LED_H
