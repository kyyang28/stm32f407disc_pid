
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "system.h"

#define MHz 1000000L
#define KHz 1000L

/* The LED indicators on the STM32F4Discovery board */
#define LED_PORT        			GPIOD
#define LED_PORT_CLOCK  			RCC_AHB1Periph_GPIOD
#define GREEN_PIN       			GPIO_Pin_12
#define ORANGE_PIN      			GPIO_Pin_13
#define RED_PIN         			GPIO_Pin_14
#define BLUE_PIN        			GPIO_Pin_15
#define ALL_LED_PINS    			GREEN_PIN | ORANGE_PIN | RED_PIN | BLUE_PIN
#define GREEN_LED       			LED_PORT,GREEN_PIN
#define ORANGE_LED      			LED_PORT,ORANGE_PIN
#define RED_LED         			LED_PORT,RED_PIN
#define BLUE_LED        			LED_PORT,BLUE_PIN
#define ALL_LEDS        			LED_PORT,ALL_LED_PINS

#define TIMER TIM4
#define TIMER_PERIPHERAL_CLOCK 		RCC_APB1Periph_TIM4
#define TIMER_AF 					GPIO_AF_TIM4

/* unfortunate globals */
//uint32_t PWM_Frequency = 1;			// Unit: Hz
uint32_t PWM_Steps = 5000;			// PWM_Steps = ARR = Period
uint32_t COUNTER_Frequency = 10000;	// 10 KHz

uint32_t ccr1, ccr2, ccr3, ccr4;

uint32_t get_timer_clock_frequency (void)
{
	uint32_t multiplier;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	
	if (RCC_Clocks.PCLK1_Frequency == RCC_Clocks.SYSCLK_Frequency) {
		multiplier = 1;
	}else {
		multiplier = 2;
	}
	
//	printf("RCC_Clocks.PCLK1_Frequency: %u, %s, %d\r\n", RCC_Clocks.SYSCLK_Frequency, __FUNCTION__, __LINE__);	// 168000000 Hz
//	printf("RCC_Clocks.PCLK1_Frequency: %u, %s, %d\r\n", RCC_Clocks.PCLK1_Frequency, __FUNCTION__, __LINE__);	// 42000000 Hz
	return multiplier * RCC_Clocks.PCLK1_Frequency;		// 2 * 42000000 = 84000000 Hz
}

void timer_clock_init (void)
{
	uint32_t TIMER_Frequency = get_timer_clock_frequency();			// TIMER_Frequency = 84000000 Hz
//	uint32_t COUNTER_Frequency = PWM_Steps * PWM_Frequency;			// COUNTER_Frequency = 10000 * 1 = 10 KHz = 10000 Hz
//	printf("TIMER_Frequency: %u, %s, %d\r\n", TIMER_Frequency, __FUNCTION__, __LINE__);
//	printf("COUNTER_Frequency: %u, %s, %d\r\n", COUNTER_Frequency, __FUNCTION__, __LINE__);

	/*
	 * Timer frequency = SYSCLK_Frequency / 2 = 168000000 / 2 = 84000000 Hz
	 * PSC value = 8400
	 * Counter frequency = Timer frequency / PSC value = 84000000 / 8400 = 10000 Hz = 10 KHz
	 * ARR value (Period or PWM steps) = 5000
	 * PWM frequency = Counter frequency / ARR value = 10000 / 5000 = 2 Hz
	 */
	uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1;	// PSC_Value = (84000000 / 10000) - 1 = 8399
	uint32_t ARR_Value = PWM_Steps - 1;								// ARR_Value = 10000 - 1 = 9999
//	printf("PSC_Value: %u, %s, %d\r\n", PSC_Value, __FUNCTION__, __LINE__);
//	printf("ARR_Value: %u, %s, %d\r\n", ARR_Value, __FUNCTION__, __LINE__);
	
	/* make sure the peripheral is clocked */
	RCC_APB1PeriphClockCmd(TIMER_PERIPHERAL_CLOCK, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  
	/* set everything back to default values */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	/* only changes from the defaults are needed */
	TIM_TimeBaseStructure.TIM_Prescaler = PSC_Value;
	TIM_TimeBaseStructure.TIM_Period = ARR_Value;
	TIM_TimeBaseStructure.TIM_CounterMode= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIMER, &TIM_TimeBaseStructure);
}

void timer_start(void)
{
	TIM_Cmd(TIMER, ENABLE);
	TIM_CtrlPWMOutputs(TIMER, ENABLE);
}

void timer_stop(void)
{
	TIM_CtrlPWMOutputs(TIMER, DISABLE);
	TIM_Cmd(TIMER, DISABLE);
}

void timer_pwm_init (void)
{
	uint32_t TimerPeriod = (PWM_Steps - 1);		// PWM_Steps(ARR value) controls the duty cycle
	// (tim2_period + 1)*Dutycycle / 100;
	ccr1 = (TimerPeriod + 1) * 50 / 100;		// Duty cycle = 50%
	ccr2 = (TimerPeriod + 1) * 75 / 100;		// Duty cycle = 33%		PD13
	ccr3 = (TimerPeriod + 1) * 25 / 100;		// Duty cycle = 25%		PD14
	ccr4 = (TimerPeriod + 1) * 20 / 100;		// Duty cycle = 20%		PD15
//	ccr1 = TimerPeriod / 2;		// Duty cycle = 50%
//	ccr2 = TimerPeriod / 3;		// Duty cycle = 33%
//	ccr3 = TimerPeriod / 4;		// Duty cycle = 25%
//	ccr4 = TimerPeriod / 5;		// Duty cycle = 20%

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	/* always initialise local variables before use */
	TIM_OCStructInit (&TIM_OCInitStructure);

	/* Common settings for all channels */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;

	/* Channel2 - ORANGE LED Pin13 (LED3) */
	TIM_OCInitStructure.TIM_Pulse = ccr2;
	TIM_OC2Init (TIMER, &TIM_OCInitStructure);

	/* Channel3 - RED LED Pin14 (LED4) */
	TIM_OCInitStructure.TIM_Pulse = ccr3;
	TIM_OC3Init (TIMER, &TIM_OCInitStructure);

	/* Channel4 - BLUE LED Pin15 */
	/* make this the opposite polarity to the other two */
	TIM_OCInitStructure.TIM_Pulse = ccr4;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC4Init (TIMER, &TIM_OCInitStructure);
}

/* these are the LEDs on the STM32F4Discovery board */
void board_leds_init (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* always initialise local variables before use */
	GPIO_StructInit(&GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(LED_PORT_CLOCK, ENABLE);

	/* these pins will be controlled by the CCRx registers */
	GPIO_InitStructure.GPIO_Pin = ORANGE_PIN + RED_PIN + BLUE_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);

	/* ensure that the pins all start off in a known state */
	GPIO_ResetBits(LED_PORT, ORANGE_PIN + RED_PIN + BLUE_PIN);

	/* this one is used with delay_ms() to act as a timing reference */
	GPIO_InitStructure.GPIO_Pin = GREEN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);

	/* The others get connected to the AF function for the timer */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, TIMER_AF);				// TIMER_AF = GPIO_AF_TIM4
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, TIMER_AF);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, TIMER_AF);
}

// return a fairly gaussian random number in the range 0-99 inclusive
int rnd (void)
{
	int intensity = 0;
	int sampleCount = 10;
	for (int i = 0; i < sampleCount; i++) {
		intensity += rand() % 100;
	}
	return intensity / sampleCount;
}

/* the flashing green LED acts as a visual timing reference */
void flash_green_led_forever (void)
{
	int brightness = 0;
	int increment = 5;
	while (1) {
		for (int i = 0; i < 20; i++) {
			delay(50);
			TIM_SetCompare3 (TIMER, rnd());
			brightness += increment;
			TIM_SetCompare2 (TIMER, brightness);
			TIM_SetCompare4 (TIMER, brightness); // note opposite polarity
		}
		increment = -increment;
		GPIO_ToggleBits (GREEN_LED);
	}
}
