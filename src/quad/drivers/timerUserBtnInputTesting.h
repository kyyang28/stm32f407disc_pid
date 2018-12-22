#ifndef __TIMERUSERBTNINPUTTESTING_H
#define __TIMERUSERBTNINPUTTESTING_H

#include <stdint.h>

extern uint8_t TIM5_CH1_CAPTURE_STATUS;			// Status of input capture
extern uint32_t TIM5_CH1_CAPTURE_VAL;				// Value of input capture (TIM5 is 32bit timer)

void TIM5_CH1_Cap_Init(uint32_t arr, uint16_t psc);

#endif	// __TIMERUSERBTNINPUTTESTING_H
