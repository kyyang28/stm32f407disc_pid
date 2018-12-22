
#include "system.h"
#include "nvic.h"
#include "atomic.h"

static uint32_t usTicks = 0;
//uint32_t usTicks = 0;		// debugging purposes, only global variable can be seen in the Watch Window in DEBUG mode
//uint32_t g_cycle_cnt;		// debugging purposes

/* current uptime for 1kHz (1ms) SysTick timer which will rollover after 49 days. */
volatile uint32_t sysTickUptime = 0;
static volatile int sysTickPending = 0;

RCC_ClocksTypeDef clocks;

void SysTick_Init(void)
{
#if defined(USE_HAL_DRIVER)
	usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
#else

	RCC_GetClocksFreq(&clocks);
	/* usTicks in MHz, i.e. microseconds */
	usTicks = clocks.SYSCLK_Frequency / 1000000;
#endif
}

void SysTick_Handler(void)
{
	//ATOMIC_BLOCK(NVIC_PRIO_MAX) {
	for ( uint8_t __ToDo = __basepriSetMemRetVal(0x10); __ToDo ; __ToDo = 0 ) {
	
	/* It seams __cleanup__ doesn't get invoked in Keil IDE */
	//for (uint8_t __basepri_save __attribute__ ((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI(), __ToDo = __basepriSetMemRetVal(0x10); __ToDo ; __ToDo = 0) {
		sysTickUptime++;
		sysTickPending = 0;
		(void)(SysTick->CTRL);
		__set_BASEPRI(0x00);		// reset basepri to 0x00, no mask for interrupt
	}
#ifdef USE_HAL_DRIVER
	HAL_IncTick();
#endif
}

/* Returns system uptime in microseconds (rollover in 70 minutes) */
uint32_t microsISR(void)
{
	register uint32_t ms, pending, cycle_cnt;
	
	for ( uint8_t __ToDo = __basepriSetMemRetVal(0x10); __ToDo ; __ToDo = 0 ) {
		
		cycle_cnt = SysTick->VAL;
		
		if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
			/*
			 *	Update pending
			 *  Record it for multiple calls within the same rollover period
			 *  (Will be cleared when served)
			 *  Note that multiple rollovers are not concerned.
			 */
			sysTickPending = 1;
			
			/* Read VAL again to ensure the value is read after the rollover. */
			cycle_cnt = SysTick->VAL;
		}
		
		ms = sysTickUptime;
		pending = sysTickPending;
		__set_BASEPRI(0x00);		// reset basepri to 0x00, no mask for interrupt
	}
	
	return ((ms + pending) * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

/* Return system uptime in milliseconds (rollover in 49 days) */
uint32_t micros(void)
{
	register uint32_t ms, cycle_cnt;
	
	/* Call microsISR() in interrupt and elevated (non-zero) BASEPRI context */
	if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI())) {
		return microsISR();
	}
	
	do {
		ms = sysTickUptime;
		cycle_cnt = SysTick->VAL;	// 0 <= cycle_cnt <= 100000 (SystemCoreClock / 1000), where SystemCoreClock = 100,000,000
		//g_cycle_cnt = cycle_cnt;
		
		/* If the SysTick timer expired during the previous instruction, we need to give it a little bit of the time
		 * for that interrupt to be delivered before we can recheck sysTickUptime.
 		 */
		__ASM volatile("\tnop\n");
	} while (ms != sysTickUptime);
	
	/* (usTicks * 1000 - cycle_cnt) / usTicks calculates the precision of micoseconds */
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;		// (ms * 1000) in microseconds, usTicks * 1000 in milliseconds
}

uint32_t millis(void)
{
	return sysTickUptime;
}

/* delay us microseconds */
void delayMicroseconds(uint32_t us)
{
	uint32_t currUs = micros();
	while ((micros() - currUs) < us);
}

/* delay ms milliseconds */
void delay(uint32_t ms)
{
	while (ms--) {
		delayMicroseconds(1000);
	}
}
