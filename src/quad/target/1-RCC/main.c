
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_flash.h"
#include "system_stm32f4xx.h"

RCC_ClocksTypeDef RCC_Clocks;
uint8_t SYSCLKSource;
uint32_t PLLCFGRVal;

extern void SetSysClock(void);

int main(void)
{
	SetSysClock();
	
	RCC_GetClocksFreq(&RCC_Clocks);
	SYSCLKSource = RCC_GetSYSCLKSource();
	
	while (1) {
	}
	
	//return 0;
}

//	/* Reset the clock to their default states */
//	RCC_DeInit();
//	
//	/* Enable the External crystal clock */
//	RCC_HSEConfig(RCC_HSE_ON);
//	
//	if (RCC_WaitForHSEStartUp() == SUCCESS) {
//		/* HSE configuration is ok */
//		RCC_PLLConfig(RCC_PLLSource_HSE, 8, 400, 4, 8);
//		RCC_PLLCmd(ENABLE);
//		
//		/* Wait till the PLL flag is set */
//		RCC_GetFlagStatus(RCC_FLAG_PLLRDY == RESET);
//		
//		/* AHB, APB1, APB2 configuration */
//		RCC_HCLKConfig(RCC_SYSCLK_Div1);
//		RCC_PCLK1Config(RCC_HCLK_Div2);
//		RCC_PCLK2Config(RCC_HCLK_Div1);

//		FLASH_SetLatency(FLASH_Latency_3);
//		
//		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//		
//		RCC_GetClocksFreq(&RCC_Clocks);
//		SYSCLKSource = RCC_GetSYSCLKSource();
//	}else {
//		/* HSE is not ready */
//		while (1);
//	}
