#ifndef __TARGET_H
#define __TARGET_H

#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

#define TARGET_IO_PORTA			0xFFFF
#define TARGET_IO_PORTB			0xFFFF
#define TARGET_IO_PORTC			0xFFFF
#define TARGET_IO_PORTD			0xFFFF

#define LED3					PD13
#define LED4					PD12
#define LED5					PD14
#define LED6					PD15

#define BTN_INT_EXTI
//#define BTN_POLL

#endif	// __TARGET_H
