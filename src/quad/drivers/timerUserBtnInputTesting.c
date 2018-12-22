
#include "timerUserBtnInputTesting.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "led.h"

TIM_ICInitTypeDef TIM5_ICInitStructure;

void TIM5_CH1_Cap_Init(uint32_t arr, uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);		// Enable TIM5 clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);		// Enable PORTA clock
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;					// GPIO A0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				// Alternate function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			// Speed 100 MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				// Push-pull alternate output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;				// Push down
	GPIO_Init(GPIOA, &GPIO_InitStructure);						// Initialise GPIO A0
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);		// Setup PA0 alternate function to TIM5
	
	/* Initialise TIME base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// prescaler of timer
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	// Up mode
	TIM_TimeBaseStructure.TIM_Period = arr;						// auto-reload value
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	/* Initialise TIM5 Input Capture Parameters */
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1;			// CC1S = 01, mapping input IC1 to TI1
	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;// Input capture occurs at rising edge of the signal
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	// map to TI1
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;		// No input divider
	TIM5_ICInitStructure.TIM_ICFilter = 0x00;					// No input filter
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	
	/* Enable update interrupt, which allows CC1IE capturing interrupt */
//	TIM_ITConfig(TIM5, TIM_IT_Update | TIM_IT_CC1, ENABLE);
	
	/* Enable TIMER5 */
	TIM_Cmd(TIM5, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	// Setup the preemption priority to 3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				// Enable IRQ channel
	NVIC_Init(&NVIC_InitStructure);								// Initialise NVIC according to above parameters
	
	/* Enable update interrupt, which allows CC1IE capturing interrupt */
	TIM_ITConfig(TIM5, TIM_IT_Update | TIM_IT_CC1, ENABLE);	
}

#if 0
/*
 * Status of Input Capture:
 * 		- [7]: 0: Capture unsuccessfully, 1: Capture successfully
 *		- [6]: 0: Low voltage level is not yet captured, 1: Low voltage level is captured successfully
*		- [5:0]: The overflow counter after low voltage level has been captured (Based on the 32-bit timer, 1us increments 1, overflow time: 4294 seconds)
 */
uint8_t TIM5_CH1_CAPTURE_STATUS = 0;			// Status of input capture
uint32_t TIM5_CH1_CAPTURE_VAL;					// Value of input capture (TIM5 is 32bit timer)

/* TIM5 Interrupt Service Routine */
void TIM5_IRQHandler(void)
{
	if ((TIM5_CH1_CAPTURE_STATUS & 0x80) == 0) {					// capture unsuccessful
		if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {		// overflow occurs
			if (TIM5_CH1_CAPTURE_STATUS & 0x40) {					// high level voltage is captured
				if ((TIM5_CH1_CAPTURE_STATUS & 0x3F) == 0x3F) {		// high level voltage is too long
					TIM5_CH1_CAPTURE_STATUS |= 0x80;				// mark capture
					TIM5_CH1_CAPTURE_VAL = 0xFFFFFFFF;
				}else {
					TIM5_CH1_CAPTURE_STATUS++;
				}
			}
		}
		
		if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) {			// capture event occurs
			if (TIM5_CH1_CAPTURE_STATUS & 0x40) {					// falling-edge is captured	(0x40: 1 << 6)
				TIM5_CH1_CAPTURE_STATUS |= 0x80;					// high level voltage is captured
				TIM5_CH1_CAPTURE_VAL = TIM_GetCapture1(TIM5);		// Obtain the current capture value from CCR1 register
				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising);	// CC1P = 0, setup rising-edge capture mode
			}else {
				TIM5_CH1_CAPTURE_STATUS = 0;						// clear capture status
				TIM5_CH1_CAPTURE_VAL = 0;							// clear capture value
				TIM5_CH1_CAPTURE_STATUS |= 0x40;					// mark rising-edge is captured
				TIM_Cmd(TIM5, DISABLE);								// disable timer5
				TIM_SetCounter(TIM5, 0);							// set CNT register of time5 to 0
				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling);// CC1P = 1, setup to falling-edge capture
				TIM_Cmd(TIM5, ENABLE);								// enable timer5
			}
		}
	}
	
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1 | TIM_IT_Update);
}
#endif
