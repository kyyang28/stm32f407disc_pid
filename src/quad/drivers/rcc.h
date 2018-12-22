#ifndef __RCC_H
#define __RCC_H

#include "utils.h"
#include "RCCTypes.h"
#include "stm32f4xx.h"

enum rcc_reg {
	RCC_EMPTY	= 0,		// make sure the default value (0) does not enable anything
//	RCC_AHB,
	RCC_APB2,		// 1
	RCC_APB1,		// 2
	RCC_AHB1		// 3
};

#define RCC_ENCODE(reg, mask)		(((reg) << 5) | LOG2_32BIT(mask))
#define RCC_APB2(periph) 			RCC_ENCODE(RCC_APB2, RCC_APB2ENR_ ## periph ## EN)
#define RCC_APB1(periph) 			RCC_ENCODE(RCC_APB1, RCC_APB1ENR_ ## periph ## EN)
#define RCC_AHB1(periph)			RCC_ENCODE(RCC_AHB1, RCC_AHB1ENR_ ## periph ## EN)

void RCC_ClockCmd(RccPeriphTag_t periphTag, FunctionalState NewState);
void RCC_ResetCmd(RccPeriphTag_t periphTag, FunctionalState NewState);

#endif	// __RCC_H
