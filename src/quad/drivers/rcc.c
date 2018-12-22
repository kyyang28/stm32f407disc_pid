
#include "rcc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"

void RCC_ClockCmd(RccPeriphTag_t periphTag, FunctionalState NewState)
{
	int tag = periphTag >> 5;		// tag = 2 for SPI2
	uint32_t mask = 1 << (periphTag & 0x1F);	// mask = 0x4000 for SPI2
	
	switch (tag) {
	case RCC_APB2:
		RCC_APB2PeriphClockCmd(mask, NewState);
		break;
	
	case RCC_APB1:		// RCC_APB1 = 2
		RCC_APB1PeriphClockCmd(mask, NewState);
		break;
	
	case RCC_AHB1:
		RCC_AHB1PeriphClockCmd(mask, NewState);
		break;
	}
}

void RCC_ResetCmd(RccPeriphTag_t periphTag, FunctionalState NewState)
{
	int tag = periphTag >> 5;
	uint32_t mask = 1 << (periphTag & 0x1F);
	
	switch (tag) {
	case RCC_APB2:
		RCC_APB2PeriphResetCmd(mask, NewState);
		break;
	
	case RCC_APB1:
		RCC_APB1PeriphResetCmd(mask, NewState);
		break;
	
	case RCC_AHB1:
		RCC_AHB1PeriphResetCmd(mask, NewState);
		break;
	}
}
