#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "stm32f4xx_rcc.h"

void SysTick_Init(void);
uint32_t micros(void);
uint32_t millis(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);
void SysTick_Handler(void);

#endif	// __SYSTEM_H
