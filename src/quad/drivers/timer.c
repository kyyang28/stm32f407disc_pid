
#include <stdio.h>				// debugging purposes ONLY
#include <string.h>
#include "timer.h"
#include "utils.h"
#include "misc.h"
#include "nvic.h"
#include "atomic.h"
#include "stm32f4xx.h"

#define TIM_N(n) 					(1 << (n))

#define USED_TIMER_COUNT 			BITCOUNT(USED_TIMERS)			// defined in timer.h
#define CC_CHANNELS_PER_TIMER 		4              					// TIM_Channel_1..4

/*
 * TIM_IT_CCx(TIM_Channel_1): (TIM_IT_CC1 << ((ch) / 4)) ==> (TIM_IT_CC1 << ((TIM_Channel_1) / 4)) ==> (0x0002 << ((0x0000) / 4)) ==> 0x0002
 * TIM_IT_CCx(TIM_Channel_2): (TIM_IT_CC1 << ((ch) / 4)) ==> (TIM_IT_CC1 << ((TIM_Channel_2) / 4)) ==> (0x0002 << ((0x0004) / 4)) ==> 0x0004
 * TIM_IT_CCx(TIM_Channel_3): (TIM_IT_CC1 << ((ch) / 4)) ==> (TIM_IT_CC1 << ((TIM_Channel_3) / 4)) ==> (0x0002 << ((0x0008) / 4)) ==> 0x0008
 * TIM_IT_CCx(TIM_Channel_4): (TIM_IT_CC1 << ((ch) / 4)) ==> (TIM_IT_CC1 << ((TIM_Channel_4) / 4)) ==> (0x0002 << ((0x000C) / 4)) ==> 0x0010
 */
#define TIM_IT_CCx(ch)				(TIM_IT_CC1 << ((ch) / 4))		// TIM_IT_CC1: 0x0002

typedef struct timerConfig_s {
    timerCCHandlerRec_t *edgeCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallback[CC_CHANNELS_PER_TIMER];
    timerOvrHandlerRec_t *overflowCallbackActive; // null-terminated linkded list of active overflow callbacks
    uint32_t forcedOverflowTimerValue;
} timerConfig_t;
timerConfig_t timerConfig[USED_TIMER_COUNT];

typedef struct {
    channelType_t type;
} timerChannelInfo_t;
timerChannelInfo_t timerChannelInfo[USABLE_TIMER_CHANNEL_COUNT];

typedef struct {
    uint8_t priority;
} timerInfo_t;
timerInfo_t timerInfo[USED_TIMER_COUNT];

#define TIMER_INDEX(i)			BITCOUNT((TIM_N(i) - 1) & USED_TIMERS)

/* +----------------------------------------------------------------------------------+ */
/* +---------------------------- Testing user button (PA0) ---------------------------+ */
/* +----------------------------------------------------------------------------------+ */
/*
 * Status of Input Capture:
 * 		- [7]: 0: Capture unsuccessfully, 1: Capture successfully
 *		- [6]: 0: Low voltage level is not yet captured, 1: Low voltage level is captured successfully
 *		- [5:0]: The overflow counter after low voltage level has been captured (Based on the 32-bit timer, 1us increments 1, overflow time: 4294 seconds)
 */
//uint8_t TIM5_CH1_CAPTURE_STATUS = 0;			// Status of input capture
//uint32_t TIM5_CH1_CAPTURE_VAL;					// Value of input capture (TIM5 is 32bit timer)
/* +----------------------------------------------------------------------------------+ */
/* +---------------------------- Testing user button (PA0) ---------------------------+ */
/* +----------------------------------------------------------------------------------+ */

static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
#define _CASE_SHF	10		// amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
#define _CASE_(tim, index)	case ((unsigned)tim >> _CASE_SHF): return index; break;
#define _CASE(i) _CASE_(TIM##i##_BASE, TIMER_INDEX(i))
	// TIM1_BASE
//	printf("TIM5_BASE: 0x%x, %s, %d\r\n", (uint32_t)TIM5_BASE, __FUNCTION__, __LINE__);		// TIM5_BASE: 0x40000C00
//	printf("tim: 0x%x, %s, %d\r\n", (uint32_t)tim, __FUNCTION__, __LINE__);					// tim: TIM5: 0x40000C00
//	printf("_CASE_SHF: %d, %s, %d\r\n", _CASE_SHF, __FUNCTION__, __LINE__);					// _CASE_SHF: 10
//	printf("TIM5_BASE >> _CASE_SHF: 0x%x, %s, %d\r\n", (uint32_t)TIM5_BASE >> _CASE_SHF, __FUNCTION__, __LINE__);	// TIM5_BASE >> _CASE_SHF: 0x100003
//	printf("tim >> _CASE_SHF: 0x%x, %s, %d\r\n", (unsigned)tim >> _CASE_SHF, __FUNCTION__, __LINE__);		// tim(TIM5) >> _CASE_SHF: 0x100003
//	printf("USED_TIMERS: 0x%x, %s, %d\r\n", USED_TIMERS, __FUNCTION__, __LINE__);

//	printf("(TIM_N(1) - 1): 0x%x, %s, %d\r\n", (TIM_N(1) - 1), __FUNCTION__, __LINE__);
//	printf("USED_TIMERS & TIM_N(1): 0x%x, %s, %d\r\n", USED_TIMERS & TIM_N(1), __FUNCTION__, __LINE__);
//	printf("(TIM_N(1) - 1) & USED_TIMERS: 0x%x, %s, %d\r\n", (TIM_N(1) - 1) & USED_TIMERS, __FUNCTION__, __LINE__);
//	printf("TIMER_INDEX(1): %d, %s, %d\r\n", TIMER_INDEX(1), __FUNCTION__, __LINE__);
//	printf("(TIM_N(2) - 1): 0x%x, %s, %d\r\n", (TIM_N(2) - 1), __FUNCTION__, __LINE__);
//	printf("(TIM_N(2) - 1) & USED_TIMERS: 0x%x, %s, %d\r\n", (TIM_N(2) - 1) & USED_TIMERS, __FUNCTION__, __LINE__);
//	printf("TIMER_INDEX(2): %d, %s, %d\r\n", TIMER_INDEX(2), __FUNCTION__, __LINE__);
//	printf("(TIM_N(3) - 1): 0x%x, %s, %d\r\n", (TIM_N(3) - 1), __FUNCTION__, __LINE__);
//	printf("(TIM_N(3) - 1) & USED_TIMERS: 0x%x, %s, %d\r\n", (TIM_N(3) - 1) & USED_TIMERS, __FUNCTION__, __LINE__);
//	printf("TIMER_INDEX(3): %d, %s, %d\r\n", TIMER_INDEX(3), __FUNCTION__, __LINE__);
//	printf("(TIM_N(5) - 1): 0x%x, %s, %d\r\n", (TIM_N(5) - 1), __FUNCTION__, __LINE__);		// (TIM_N(5) - 1) = 0x1f (31)
//	printf("(TIM_N(5) - 1) & USED_TIMERS: 0x%x, %s, %d\r\n", (TIM_N(5) - 1) & USED_TIMERS, __FUNCTION__, __LINE__);	// (TIM_N(5) - 1) & USED_TIMERS = 0x1e
//	printf("TIMER_INDEX(5): %d, %s, %d\r\n", TIMER_INDEX(5), __FUNCTION__, __LINE__);			// TIMER_INDEX(5) = 4
	
	/* let gcc do the work, switch should be quite optimised */
	switch ((unsigned)tim >> _CASE_SHF) {
#if USED_TIMERS & TIM_N(1)
		_CASE(1);
#endif
#if USED_TIMERS & TIM_N(2)
		_CASE(2);
#endif
#if USED_TIMERS & TIM_N(3)
		_CASE(3);
#endif
#if USED_TIMERS & TIM_N(4)
		_CASE(4);
#endif
#if USED_TIMERS & TIM_N(5)
		_CASE(5);
#endif
#if USED_TIMERS & TIM_N(6)
		_CASE(6);
#endif
#if USED_TIMERS & TIM_N(7)
		_CASE(7);
#endif
#if USED_TIMERS & TIM_N(8)
		_CASE(8);
#endif
#if USED_TIMERS & TIM_N(9)
		_CASE(9);
#endif
#if USED_TIMERS & TIM_N(10)
		_CASE(10);
#endif
#if USED_TIMERS & TIM_N(11)
		_CASE(11);
#endif
#if USED_TIMERS & TIM_N(12)
		_CASE(12);
#endif
#if USED_TIMERS & TIM_N(13)
		_CASE(13);
#endif
#if USED_TIMERS & TIM_N(14)
		_CASE(14);
#endif
#if USED_TIMERS & TIM_N(15)
		_CASE(15);
#endif
#if USED_TIMERS & TIM_N(16)
		_CASE(16);
#endif
#if USED_TIMERS & TIM_N(17)
		_CASE(17);
#endif
	default: return ~1;		// make sure final index is out of range
	}
#undef _CASE
#undef _CASE_
}

static inline uint8_t lookupChannelIndex(const uint16_t channel)
{
	/*
	 *	#define TIM_Channel_1                      ((uint16_t)0x0000)
	 *  #define TIM_Channel_2                      ((uint16_t)0x0004)
	 *	#define TIM_Channel_3                      ((uint16_t)0x0008)
	 *	#define TIM_Channel_4                      ((uint16_t)0x000C)
	 *
	 * channel = TIM_Channel_1 or TIM_Channel_2 or TIM_Channel_3 or TIM_Channel_4
	 * if channel == TIM_Channel_1
	 *		channel >> 2: TIM_Channel_1 >> 2: 0x0000 >> 2: 0x0
	 * 
	 * if channel == TIM_Channel_2
	 *		channel >> 2: TIM_Channel_2 >> 2: 0x0004 >> 2: 0x1
	 * 
	 * if channel == TIM_Channel_3
	 *		channel >> 2: TIM_Channel_3 >> 2: 0x0008 >> 2: 0x2
	 * 
	 * if channel == TIM_Channel_4
	 *		channel >> 2: TIM_Channel_4 >> 2: 0x000C >> 2: 0x3
	 *
	 * channelIndex:
	 *		- 0 corresponds to TIM_Channel_1
	 *		- 1 corresponds to TIM_Channel_2
	 *		- 2 corresponds to TIM_Channel_3
	 *		- 3 corresponds to TIM_Channel_4
 	 */
	return channel >> 2;
}

RccPeriphTag_t timerRCC(TIM_TypeDef *tim)
{
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].TIMx == tim) {
            return timerDefinitions[i].rcc;
        }
    }
    return 0;
}

void timerForceOverflow(TIM_TypeDef *tim)
{
	uint8_t timerIndex = lookupTimerIndex((const TIM_TypeDef *)tim);
	
	/* 0x50 = NVIC_PRIO_TIMER defined in nvic.h */
	for ( uint8_t __ToDo = __basepriSetMemRetVal(0x50); __ToDo ; __ToDo = 0 ) {
		/* Save the current count so that PPM reading will work on the same timer that was forced to overflow */
		timerConfig[timerIndex].forcedOverflowTimerValue = tim->CNT + 1;
		
		/* Force an overflow by setting the UG bit */
		tim->EGR |= TIM_EGR_UG;
		__set_BASEPRI(0x00);		// reset basepri to 0x00, no mask for interrupt
	}
}

const timerHardware_t *timerGetByTag(ioTag_t tag, timerUsageFlag_e flag)
{
	if (!tag) {
		return NULL;
	}
	
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
		if (timerHardware[i].tag == tag) {
			if (timerHardware[i].usageFlags & flag || flag == 0) {
				return &timerHardware[i];
			}
		}
	}
	
	return NULL;
}

void configTimeBaseKhz(TIM_TypeDef *tim, uint16_t period, uint8_t khz)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xFFFF;		// AKA TIMx_ARR
	
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / timerClockDivisor(tim) / ((uint32_t)khz * 1000)) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

void configTimeBaseMhz(TIM_TypeDef *tim, uint16_t period, uint8_t Mhz)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xFFFF;		// AKA TIMx_ARR
//	printf("TIM_TimeBaseStructure.TIM_Period: %u, %s, %d\r\n", TIM_TimeBaseStructure.TIM_Period, __FUNCTION__, __LINE__);
	
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / timerClockDivisor(tim) / ((uint32_t)Mhz * 1000000)) - 1;
//	printf("TIM_TimeBaseStructure.TIM_Prescaler: %u, %s, %d\r\n", TIM_TimeBaseStructure.TIM_Prescaler, __FUNCTION__, __LINE__);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

void configTimeBaseMhz4UserBtn(TIM_TypeDef *tim, uint32_t period, uint8_t Mhz)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	printf("period: 0x%x, %s, %d\r\n", period, __FUNCTION__, __LINE__);
	TIM_TimeBaseStructure.TIM_Period = period;		// AKA TIMx_ARR
//	TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xFFFF;		// AKA TIMx_ARR
	
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / timerClockDivisor(tim) / ((uint32_t)Mhz * 1000000)) - 1;
//	printf("TIM_Prescaler: %u, %s, %d\r\n", TIM_TimeBaseStructure.TIM_Prescaler, __FUNCTION__, __LINE__);
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init)
{
	switch (channel) {
		case TIM_Channel_1:
			TIM_OC1Init(tim, init);
			break;
		case TIM_Channel_2:
			TIM_OC2Init(tim, init);
			break;
		case TIM_Channel_3:
			TIM_OC3Init(tim, init);
			break;
		case TIM_Channel_4:
			TIM_OC4Init(tim, init);
			break;
	}
}

void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload)
{
	switch (channel) {
		case TIM_Channel_1:
			TIM_OC1PreloadConfig(tim, preload);
			break;
		case TIM_Channel_2:
			TIM_OC2PreloadConfig(tim, preload);
			break;
		case TIM_Channel_3:
			TIM_OC3PreloadConfig(tim, preload);
			break;
		case TIM_Channel_4:
			TIM_OC4PreloadConfig(tim, preload);
			break;
	}
}

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw)
{
	return (volatile timCCR_t *)((volatile char *)&timHw->tim->CCR1 + timHw->channel);
}

void timerInit(void)
{
	memset(timerConfig, 0, sizeof(timerConfig));
	
	/* Enable the timer peripherals */
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
		RCC_ClockCmd(timerRCC(timerHardware[i].tim), ENABLE);
	}
	
#if defined(STM32F4)
	for (int timerIndex = 0; timerIndex < USABLE_TIMER_CHANNEL_COUNT; timerIndex++) {
		const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];
		if (timerHardwarePtr->usageFlags == TIM_USE_PWM) {			// if timer uses PWM input capture mode, initialise to push-down configuration
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//			printf("timerHardwarePtr->alternateFunction: %u, %s, %d\r\n", timerHardwarePtr->alternateFunction, __FUNCTION__, __LINE__);
			IOConfigGPIOAF(IOGetByTag(timerHardwarePtr->tag), IOCFG_AF_PP_PD, timerHardwarePtr->alternateFunction);
		}else {
			IOConfigGPIOAF(IOGetByTag(timerHardwarePtr->tag), IOCFG_AF_PP, timerHardwarePtr->alternateFunction);
		}
	}
#endif
	
	/* Initialise timer channel structures */
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
		timerChannelInfo[i].type = TYPE_FREE;
	}
	
	for (int i = 0; i < USED_TIMER_COUNT; i++) {
		timerInfo[i].priority = ~0;
	}
}

uint8_t timerInputIrq(TIM_TypeDef *tim)
{
	for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
		if (timerDefinitions[i].TIMx == tim) {
			return timerDefinitions[i].inputIrq;
		}
	}
	return 0;
}

void timerNVICConfigure(uint8_t irq)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
//	printf("irq: %u, %s, %d\r\n", irq, __FUNCTION__, __LINE__);		// irq (TIM5_IRQn) = 50
	NVIC_InitStructure.NVIC_IRQChannel = irq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER);
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER);
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/* Old interface for PWM inputs. It should be replaced */
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz)
{
	/* Timer base configuration */
	configTimeBaseMhz(timerHardwarePtr->tim, period, mhz);
	TIM_Cmd(timerHardwarePtr->tim, ENABLE);				// Enable TIM5
	
	/* Timer NVIC IRQ configuration */
	uint8_t irq = timerInputIrq(timerHardwarePtr->tim);
	timerNVICConfigure(irq);
	
	/* HACK - enable second IRQ on timers that need it */
	switch (irq) {
		case TIM1_CC_IRQn:
			timerNVICConfigure(TIM1_UP_TIM10_IRQn);
			break;
		
		case TIM8_CC_IRQn:
			timerNVICConfigure(TIM8_UP_TIM13_IRQn);
			break;
	}
}

/* Old interface for PWM inputs. It should be replaced */
void timerConfigure4UserBtn(const timerHardware_t *timerHardwarePtr, uint32_t period, uint8_t mhz)
{
	/* Timer base configuration */
	configTimeBaseMhz4UserBtn(timerHardwarePtr->tim, period, mhz);
	TIM_Cmd(timerHardwarePtr->tim, ENABLE);				// Enable TIM5
	
	/* Timer NVIC IRQ configuration */
	uint8_t irq = timerInputIrq(timerHardwarePtr->tim);
	timerNVICConfigure(irq);

	/* HACK - enable second IRQ on timers that need it */
	switch (irq) {
		case TIM1_CC_IRQn:
			timerNVICConfigure(TIM1_UP_TIM10_IRQn);
			break;
		
		case TIM8_CC_IRQn:
			timerNVICConfigure(TIM8_UP_TIM13_IRQn);
			break;
	}
}

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn)
{
	self->fn = fn;
}

void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn)
{
	self->fn = fn;
	self->next = NULL;
}

/* Update overflow callback list
 * some synchronisation mechanism is neccesary to avoid disturbing other channels (BASEPRI used now)
 */
static void timerChConfig_UpdateOverflow(timerConfig_t *cfg, TIM_TypeDef *tim)
{
	timerOvrHandlerRec_t **chain = &cfg->overflowCallbackActive;
	
	/* It seems __cleanup__ doesn't get invoked in Keil IDE */
	//for (uint8_t __basepri_save __attribute__ ((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI(), __ToDo = __basepriSetMemRetVal(NVIC_PRIO_TIMER); __ToDo ; __ToDo = 0) {
//	ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
	for ( uint8_t __ToDo = __basepriSetMemRetVal(NVIC_PRIO_TIMER); __ToDo ; __ToDo = 0 ) {
		for (int i = 0; i < CC_CHANNELS_PER_TIMER; i++)
			if (cfg->overflowCallback[i]) {
				*chain = cfg->overflowCallback[i];
				chain = &cfg->overflowCallback[i]->next;
			}
		*chain = NULL;
		__set_BASEPRI(0x00);		// reset basepri to 0x00, no mask for interrupt
	}
	
	/* enable or disable IRQ */
//	printf("cfg->overflowCallbackActive: 0x%x, %s, %d\r\n", (uint32_t)cfg->overflowCallbackActive, __FUNCTION__, __LINE__);	// (uint32_t)cfg->overflowCallbackActive = 0x200017dc
	TIM_ITConfig(tim, TIM_IT_Update, cfg->overflowCallbackActive ? ENABLE : DISABLE);	// ENABLE TIM_IT_Update timer overflow interrupt here
}

/* config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive */
void timerChConfigCallbacks(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback)
{
	uint8_t timerIndex = lookupTimerIndex(timHw->tim);
//	printf("timHw->tim: 0x%x, %s, %d\r\n", (uint32_t)timHw->tim, __FUNCTION__, __LINE__);	// timHw->tim = 0x40000c00 (TIM5)
//	printf("timerIndex: %u, %s, %d\r\n", timerIndex, __FUNCTION__, __LINE__);				// timerIndex = 4
	if (timerIndex > USED_TIMER_COUNT) {
		return;
	}
	
//	printf("timHw->channel: %u, %s, %d\r\n", timHw->channel, __FUNCTION__, __LINE__);	// timHw->channel = 0 (TIM5 channel 1)
	uint8_t channelIndex = lookupChannelIndex(timHw->channel);
//	printf("channelIndex: %u, %s, %d\r\n", channelIndex, __FUNCTION__, __LINE__);
	
	if (edgeCallback == NULL) {		// disable irq before changing callback to NULL
		TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), DISABLE);
	}
	
	/* Setup callback info */
	timerConfig[timerIndex].edgeCallback[channelIndex] = edgeCallback;
	timerConfig[timerIndex].overflowCallback[channelIndex] = overflowCallback;
	
	/* Enable channel IRQ */
	if (edgeCallback) {
		TIM_ITConfig(timHw->tim, TIM_IT_CCx(timHw->channel), ENABLE);			// ENABLE TIM_IT_CC1 timer compare interrupt here
	}
	
	timerChConfig_UpdateOverflow(&timerConfig[timerIndex], timHw->tim);			// ENABLE TIM_IT_Update timer overflow interrupt in this function
}

/* Timer Input Capture Interrupt Service Routine (ISR) function */
static void timCCxHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
{
	uint16_t capture;
//	unsigned tim_status_sr, tim_status_dier;
	unsigned tim_status;
//	tim_status_sr = tim->SR;
//	tim_status_dier = tim->DIER;
	tim_status = tim->SR & tim->DIER;
//	printf("sr: %u\r\n", tim_status_sr);
//	printf("dier: %u\r\n", tim_status_dier);
//	printf("tim_status: %u\r\n", tim_status);

#if 1
	while (tim_status) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		/* flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
		 * current order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
		 */
		unsigned bit = __builtin_clz(tim_status);		// __builtin_clz returns the leading number of zero bits starting from MSB
		unsigned mask = ~(0x80000000 >> bit);
//		printf("bit: %u\r\n", bit);
//		printf("mask: %u\r\n", mask);
		tim->SR = mask;						// Clears the TIMx's interrupt pending bits
		tim_status &= mask;
		
		switch (bit) {
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
#if 0
//			/* timer overflow handler */
//			case __builtin_clz(TIM_IT_Update): {	// __builtin_clz(TIM_IT_Update): 31
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//				break;
//			}
//			
//			/* timer capture compare 1 handler */
//			case __builtin_clz(TIM_IT_CC1):			// __builtin_clz(TIM_IT_CC1): 30
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//				timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);
//				break;

//			/* timer capture compare 2 handler */
//			case __builtin_clz(TIM_IT_CC2):			// __builtin_clz(TIM_IT_CC2): 29
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//				timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
//				break;

//			/* timer capture compare 3 handler */
//			case __builtin_clz(TIM_IT_CC3):			// __builtin_clz(TIM_IT_CC3): 28
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//				timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
//				break;

//			/* timer capture compare 4 handler */
//			case __builtin_clz(TIM_IT_CC4):			// __builtin_clz(TIM_IT_CC4): 27
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//				timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
//				break;
#else
			/* timer overflow handler */
			case 31: {	// __builtin_clz(TIM_IT_Update) = 31
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//				printf("timerConfig->forcedOverflowTimerValue: %u, %s, %d\r\n", timerConfig->forcedOverflowTimerValue, __FUNCTION__, __LINE__);
				if (timerConfig->forcedOverflowTimerValue != 0) {			// forcedOverflowTimerValue used in pwm_output.c oneshot125 motor protocol
					capture = timerConfig->forcedOverflowTimerValue - 1;
					timerConfig->forcedOverflowTimerValue = 0;
				}else {
					capture = tim->ARR;
//					printf("capture: %u, %s, %d\r\n", capture, __FUNCTION__, __LINE__);
				}
				
				timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
				while (cb) {
					cb->fn(cb, capture);		// call pwmOverflowCallback() function in rx_pwm.c
					cb = cb->next;
				}
				break;
			}
			
			/* timer capture compare 1 handler */
			case 30:			// __builtin_clz(TIM_IT_CC1) = 30
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
				timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);		// call pwmEdgeCallback() function in rx_pwm.c
				break;

			/* timer capture compare 2 handler */
			case 29:			// __builtin_clz(TIM_IT_CC2) = 29
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
				timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
				break;

			/* timer capture compare 3 handler */
			case 28:			// __builtin_clz(TIM_IT_CC3) = 28
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
				timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
				break;

			/* timer capture compare 4 handler */
			case 27:			// __builtin_clz(TIM_IT_CC4) = 27
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
				timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
				break;
#endif
		}	// end switch
	}
#endif
}

/* Handler for shared interrupts when both timers need to check status bits
 * TIMER_INDEX(1) = 0
 * TIMER_INDEX(2) = 1
 * TIMER_INDEX(3) = 2
 * TIMER_INDEX(4) = 3
 * TIMER_INDEX(5) = 4
 */
#define _TIM_IRQ_HANDLER(name, i)									\
	void name(void)													\
	{																\
		timCCxHandler(TIM ## i, &timerConfig[TIMER_INDEX(i)]);		\
	} struct dummy

#if USED_TIMERS & TIM_N(1)
_TIM_IRQ_HANDLER(TIM1_CC_IRQHandler, 1);
#endif

#if USED_TIMERS & TIM_N(2)
_TIM_IRQ_HANDLER(TIM2_IRQHandler, 2);
#endif

#if USED_TIMERS & TIM_N(3)
_TIM_IRQ_HANDLER(TIM3_IRQHandler, 3);
#endif

#if USED_TIMERS & TIM_N(4)
_TIM_IRQ_HANDLER(TIM4_IRQHandler, 4);
#endif

#if 1
#if USED_TIMERS & TIM_N(5)
_TIM_IRQ_HANDLER(TIM5_IRQHandler, 5);
#endif
#else
/* TIM5 Interrupt Service Routine */
//void TIM5_IRQHandler(void)
//{
//	if ((TIM5_CH1_CAPTURE_STATUS & 0x80) == 0) {					// capture unsuccessful
//		if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {		// overflow occurs
//			if (TIM5_CH1_CAPTURE_STATUS & 0x40) {					// high level voltage is captured
//				if ((TIM5_CH1_CAPTURE_STATUS & 0x3F) == 0x3F) {		// high level voltage is too long
//					TIM5_CH1_CAPTURE_STATUS |= 0x80;				// mark capture
//					TIM5_CH1_CAPTURE_VAL = 0xFFFFFFFF;
//				}else {
//					TIM5_CH1_CAPTURE_STATUS++;
//				}
//			}
//		}
//		
//		if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) {			// capture event occurs
//			if (TIM5_CH1_CAPTURE_STATUS & 0x40) {					// falling-edge is captured	(0x40: 1 << 6)
//				TIM5_CH1_CAPTURE_STATUS |= 0x80;					// high level voltage is captured
//				TIM5_CH1_CAPTURE_VAL = TIM_GetCapture1(TIM5);		// Obtain the current capture value from CCR1 register
//				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising);	// CC1P = 0, setup rising-edge capture mode
//			}else {
//				TIM5_CH1_CAPTURE_STATUS = 0;						// clear capture status
//				TIM5_CH1_CAPTURE_VAL = 0;							// clear capture value
//				TIM5_CH1_CAPTURE_STATUS |= 0x40;					// mark rising-edge is captured
//				TIM_Cmd(TIM5, DISABLE);								// disable timer5
//				TIM_SetCounter(TIM5, 0);							// set CNT register of time5 to 0
//				TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling);// CC1P = 1, setup to falling-edge capture
//				TIM_Cmd(TIM5, ENABLE);								// enable timer5
//			}
//		}
//	}
//	
//	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1 | TIM_IT_Update);
//}
#endif

#if 0
//static void timCCxHandler(TIM_TypeDef *tim, timerConfig_t *timerConfig)
//{
//	uint16_t capture;
////	unsigned tim_status_sr, tim_status_dier;
//	unsigned tim_status;
////	tim_status_sr = tim->SR;
////	tim_status_dier = tim->DIER;
//	tim_status = tim->SR & tim->DIER;
////	printf("sr: %u\r\n", tim_status_sr);
////	printf("dier: %u\r\n", tim_status_dier);
////	printf("tim_status: %u\r\n", tim_status);

//#if 1
//	while (tim_status) {
////		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//		/* flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
//		 * current order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
//		 */
//		unsigned bit = __builtin_clz(tim_status);
//		unsigned mask = ~(0x80000000 >> bit);
////		printf("bit: %u\r\n", bit);
////		printf("mask: %u\r\n", mask);
//		tim->SR = mask;						// Clears the TIMx's interrupt pending bits
//		tim_status &= mask;
//		
//		if ((TIM5_CH1_CAPTURE_STATUS & 0x80) == 0) {
//			switch (bit) {
//	//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//	#if 0
//	//			/* timer overflow handler */
//	//			case __builtin_clz(TIM_IT_Update): {	// __builtin_clz(TIM_IT_Update): 31
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//	//				break;
//	//			}
//	//			
//	//			/* timer capture compare 1 handler */
//	//			case __builtin_clz(TIM_IT_CC1):			// __builtin_clz(TIM_IT_CC1): 30
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//	//				timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);
//	//				break;

//	//			/* timer capture compare 2 handler */
//	//			case __builtin_clz(TIM_IT_CC2):			// __builtin_clz(TIM_IT_CC2): 29
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//	//				timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
//	//				break;

//	//			/* timer capture compare 3 handler */
//	//			case __builtin_clz(TIM_IT_CC3):			// __builtin_clz(TIM_IT_CC3): 28
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//	//				timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
//	//				break;

//	//			/* timer capture compare 4 handler */
//	//			case __builtin_clz(TIM_IT_CC4):			// __builtin_clz(TIM_IT_CC4): 27
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//	//				timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
//	//				break;
//	#else
//				/* timer overflow handler */
//				case 31: {	// __builtin_clz(TIM_IT_Update) = 31
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//	//				printf("timerConfig->forcedOverflowTimerValue: %u, %s, %d\r\n", timerConfig->forcedOverflowTimerValue, __FUNCTION__, __LINE__);
//					if (timerConfig->forcedOverflowTimerValue != 0) {			// forcedOverflowTimerValue used in pwm_output.c oneshot125 motor protocol
//						capture = timerConfig->forcedOverflowTimerValue - 1;
//						timerConfig->forcedOverflowTimerValue = 0;
//					}else {
//						capture = tim->ARR;
//	//					printf("capture: %u, %s, %d\r\n", capture, __FUNCTION__, __LINE__);
//					}
//					
//					timerOvrHandlerRec_t *cb = timerConfig->overflowCallbackActive;
//					while (cb) {
//						cb->fn(cb, capture);		// call pwmOverflowCallback() function in rx_pwm.c
//						cb = cb->next;
//					}
//					break;
//				}
//				
//				/* timer capture compare 1 handler */
//				case 30:			// __builtin_clz(TIM_IT_CC1) = 30
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//					timerConfig->edgeCallback[0]->fn(timerConfig->edgeCallback[0], tim->CCR1);		// call pwmEdgeCallback() function in rx_pwm.c
//					break;

//				/* timer capture compare 2 handler */
//				case 29:			// __builtin_clz(TIM_IT_CC2) = 29
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//					timerConfig->edgeCallback[1]->fn(timerConfig->edgeCallback[1], tim->CCR2);
//					break;

//				/* timer capture compare 3 handler */
//				case 28:			// __builtin_clz(TIM_IT_CC3) = 28
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//					timerConfig->edgeCallback[2]->fn(timerConfig->edgeCallback[2], tim->CCR3);
//					break;

//				/* timer capture compare 4 handler */
//				case 27:			// __builtin_clz(TIM_IT_CC4) = 27
//	//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//					timerConfig->edgeCallback[3]->fn(timerConfig->edgeCallback[3], tim->CCR4);
//					break;
//	#endif
//			}	// end switch
//		}	// end if ((TIM5_CH1_CAPTURE_STATUS & 0x80) == 0)
//	}
//#endif
//}
#endif
