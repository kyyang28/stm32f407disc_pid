
#include <stdio.h>			// debugging purposes
#include "rx_pwm.h"
#include "timer.h"
#include "pwm_output.h"

#define MAX_MISSED_PWM_EVENTS 				10

//#define PPM_CAPTURE_COUNT 				12

//#if PPM_CAPTURE_COUNT > PWM_INPUT_PORT_COUNT
//#define PWM_PORTS_OR_PPM_CAPTURE_COUNT			PPM_CAPTURE_COUNT
//#else
//#define PWM_PORTS_OR_PPM_CAPTURE_COUNT			PWM_INPUT_PORT_COUNT
//#endif

#define PWM_PORTS_OR_PPM_CAPTURE_COUNT			PWM_INPUT_PORT_COUNT

// TODO - change to timer clocks ticks
#define INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX 		0x03

//#define PWM_TIMER_PERIOD		0xFFFFFFFF			// for PA0 button testing
#define PWM_TIMER_PERIOD		0x10000				// 0x10000 = 65536

static inputFilteringMode_e inputFilteringMode;

typedef enum {
	INPUT_MODE_PPM,
	INPUT_MODE_PWM
}pwmInputMode_t;

typedef struct {
	pwmInputMode_t mode;
	uint8_t channel;			// only used for pwm, ignored by ppm

	uint8_t state;
	/* TODO: stick with uint32_t for rise, fall and capture value, need to figure out why betaflight uses captureCompare_t(uint16_t)
	 * According to <RM0090-STM32F407-Reference_manual.pdf> 18.4.13-18.4.16 TIMx capture/compare register 1 (TIMx_CCR1) to TIMx capture/compare register 4 (TIMx_CCR4)
	 * CCRx could be 16-bit or 32-bit registers (depending on timers)
 	 */
//	timCCR_t rise;				// timCCR_t is uint32_t
//	timCCR_t fall;
//	timCCR_t capture;
	captureCompare_t rise;		// captureCompare_t is uint16_t
	captureCompare_t fall;
	captureCompare_t capture;

	uint8_t missedEvents;
	
	const timerHardware_t *timerHardware;
	timerCCHandlerRec_t edgeCb;
	timerOvrHandlerRec_t overflowCb;
}pwmInputPort_t;

static pwmInputPort_t pwmInputPorts[PWM_INPUT_PORT_COUNT];

//static uint32_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];			// For STM32F407 Discovery, only TIM2 and TIM5 use 32-bit ARR register
static uint16_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = channel;
	TIM_ICInitStructure.TIM_ICPolarity = polarity;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	
	if (inputFilteringMode == INPUT_FILTERING_ENABLED) {
		TIM_ICInitStructure.TIM_ICFilter = INPUT_FILTER_TO_HELP_WITH_NOISE_FROM_OPENLRS_TELEMETRY_RX;
	}else {
		TIM_ICInitStructure.TIM_ICFilter = 0x00;
	}
	
	TIM_ICInit(tim, &TIM_ICInitStructure);
}

/* +----------------------------------------------------------------------------------+ */
/* +---------------------------- Testing user button (PA0) ---------------------------+ */
/* +----------------------------------------------------------------------------------+ */
/*
 * Status of Input Capture:
 * 		- [7]: 0: Capture unsuccessfully, 1: Capture successfully
 *		- [6]: 0: Low voltage level is not yet captured, 1: Low voltage level is captured successfully
 *		- [5:0]: The overflow counter after low voltage level has been captured (Based on the 32-bit timer, 1us increments 1, overflow time: 4294 seconds)
 */
//uint8_t TIM_CAPTURE_STATUS = 0;			// status of timer input capture
/* +----------------------------------------------------------------------------------+ */
/* +---------------------------- Testing user button (PA0) ---------------------------+ */
/* +----------------------------------------------------------------------------------+ */

static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, timCCR_t capture)			// TODO: change back to captureCompare_t capture parameter later
//static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
//	static uint32_t cnt = 0;
//	printf("cnt: %d, %s, %d\r\n", cnt++, __FUNCTION__, __LINE__);
	pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, edgeCb);
//	printf("cbRec: 0x%x, %s, %d\r\n", (uint32_t)cbRec, __FUNCTION__, __LINE__);
//	printf("pwmInputPort: 0x%x, %s, %d\r\n", (uint32_t)pwmInputPort, __FUNCTION__, __LINE__);
	const timerHardware_t *timerHardwarePtr = pwmInputPort->timerHardware;
//	printf("timerHardwarePtr->tim: 0x%x, %s, %d\r\n", (uint32_t)timerHardwarePtr->tim, __FUNCTION__, __LINE__);						// 0x40000c00: TIM5 address
//	printf("timerHardwarePtr->channel: %u, %s, %d\r\n", timerHardwarePtr->channel, __FUNCTION__, __LINE__);							// 0x0: TIM_Channel_1
//	printf("timerHardwarePtr->tag: 0x%x, %s, %d\r\n", timerHardwarePtr->tag, __FUNCTION__, __LINE__);								// 0x10: PA0
//	printf("timerHardwarePtr->usageFlags: 0x%x, %s, %d\r\n", timerHardwarePtr->usageFlags, __FUNCTION__, __LINE__);					// 0x2: TIM_USE_PWM
//	printf("timerHardwarePtr->output: 0x%x, %s, %d\r\n", timerHardwarePtr->output, __FUNCTION__, __LINE__);							// 0x1
//	printf("timerHardwarePtr->alternateFunction: 0x%x, %s, %d\r\n", timerHardwarePtr->alternateFunction, __FUNCTION__, __LINE__);	// 0x2: GPIO_AF_TIM5

#if 1
	if (pwmInputPort->state == 0) {			// rising edge captured
		pwmInputPort->rise = capture;
//		printf("pwmInputPort->rise: %u, %s, %d\r\n", pwmInputPort->rise, __FUNCTION__, __LINE__);
		pwmInputPort->state = 1;			// change state for falling edge capture
		pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Falling);
//		TIM_CAPTURE_STATUS = 0x0;			// clear capture status
//		TIM_CAPTURE_STATUS |= 0x40;			// 0x40 = (1 << 6), falling edge is not captured
	}else {									// falling edge captured
		pwmInputPort->fall = capture;
//		printf("pwmInputPort->fall: %u, %s, %d\r\n", pwmInputPort->fall, __FUNCTION__, __LINE__);
		
		/* compute and store capture */
		pwmInputPort->capture = pwmInputPort->fall - pwmInputPort->rise;
//		printf("pwmInputPort->capture: %u, %s, %d\r\n", pwmInputPort->capture, __FUNCTION__, __LINE__);		
//		printf("pwmInputPort->channel: %u, %s, %d\r\n", pwmInputPort->channel, __FUNCTION__, __LINE__);		
		captures[pwmInputPort->channel] = pwmInputPort->capture;		// store the time difference (in us) between the falling and rising edges to captures array
		
//		TIM_CAPTURE_STATUS |= 0x80;					// 0x80 = (1 << 7), capture successfully
//		printf("TIM_CAPTURE_STATUS: 0x%x, %s, %d\r\n", TIM_CAPTURE_STATUS, __FUNCTION__, __LINE__);
		
		/* switch state */
		pwmInputPort->state = 0;			// change state for rising edge capture
		pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);
		pwmInputPort->missedEvents = 0;
	}
#else
	if (TIM5_CH1_CAPTURE_STATUS & 0x40) {					// falling-edge is captured	(0x40: 1 << 6)
		TIM5_CH1_CAPTURE_STATUS |= 0x80;					// high level voltage is captured
		TIM5_CH1_CAPTURE_VAL = TIM_GetCapture1(TIM5);		// Obtain the current capture value from CCR1 register
//		printf("TIM5_CH1_CAPTURE_VAL: %u\r\n", TIM5_CH1_CAPTURE_VAL);
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
#endif
}

static void pwmOverflowCallback(timerOvrHandlerRec_t *cbRec, timCCR_t capture)			// TODO: change back to captureCompare_t capture parameter later
//static void pwmOverflowCallback(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
//	printf("capture: %u, %s, %d\r\n", capture, __FUNCTION__, __LINE__);

#if 1
	UNUSED(capture);
	pwmInputPort_t *pwmInputPort = container_of(cbRec, pwmInputPort_t, overflowCb);
	
	if (++pwmInputPort->missedEvents > MAX_MISSED_PWM_EVENTS) {
		captures[pwmInputPort->channel] =  PPM_RCVR_TIMEOUT;
		pwmInputPort->missedEvents = 0;
	}
#else
	if (TIM5_CH1_CAPTURE_STATUS & 0x40) {					// high level voltage is captured
		if ((TIM5_CH1_CAPTURE_STATUS & 0x3F) == 0x3F) {		// high level voltage is too long
			TIM5_CH1_CAPTURE_STATUS |= 0x80;				// mark capture
			TIM5_CH1_CAPTURE_VAL = 0xFFFFFFFF;
		}else {
			TIM5_CH1_CAPTURE_STATUS++;
		}
	}
#endif	
}

void pwmRxInit(const pwmConfig_t *pwmConfig)
{
#if 1	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	inputFilteringMode = pwmConfig->inputFilteringMode;
	
	for (int channel = 0; channel < PWM_INPUT_PORT_COUNT; channel++) {
		pwmInputPort_t *port = &pwmInputPorts[channel];
		
		const timerHardware_t *timer = timerGetByTag(pwmConfig->ioTags[channel], TIM_USE_ANY);
		
		if (!timer) {
			/* TODO: maybe fail here if not enough channels? */
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			continue;
		}
		
//		printf("timer->tim: 0x%x, %s, %d\r\n", (uint32_t)timer->tim, __FUNCTION__, __LINE__);					// 0x40000000: TIM2 address
//		printf("timer->channel: 0x%x, %s, %d\r\n", timer->channel, __FUNCTION__, __LINE__);						// 0x4: TIM_Channel_2
//		printf("timer->tag: 0x%x, %s, %d\r\n", timer->tag, __FUNCTION__, __LINE__);								// 0x11: PA1
//		printf("timer->usageFlags: 0x%x, %s, %d\r\n", timer->usageFlags, __FUNCTION__, __LINE__);				// 0x2: TIM_USE_PWM
//		printf("timer->output: 0x%x, %s, %d\r\n", timer->output, __FUNCTION__, __LINE__);						// 0x0
//		printf("timer->alternateFunction: 0x%x, %s, %d\r\n", timer->alternateFunction, __FUNCTION__, __LINE__);	// 0x1: GPIO_AF_TIM2
		
		port->state = 0;
		port->missedEvents = 0;
		port->channel = channel;
		port->mode = INPUT_MODE_PWM;
		port->timerHardware = timer;
		
		/* GPIO and timer clocks are initialised in systemInit(), timerInit() and IOConfigGPIOAF() functions */
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);		// Enable TIM5 clock
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);		// Enable PORTA clock
		
		/* IO configuration */
		IO_t io = IOGetByTag(pwmConfig->ioTags[channel]);
//		printf("IO_GPIO(io): 0x%x\r\n", (uint32_t)IO_GPIO(io));		// gpio = 0x40020000 (GPIOA)
//		printf("IO_Pin(io): %u\r\n", IO_Pin(io));						// pin = 1 = 0x1 (1 << 0)
		
		IOInit(io, OWNER_PWMINPUT, RESOURCE_INDEX(channel));
//		IOConfigGPIO(io, IOCFG_AF_PP_PD);			// for testing user button (PA0) ONLY, doesn't work with out AF
//		IOConfigGPIO(io, IOCFG_AF_PP);
		
		// TODO: need to figure out WHY reinitialisation is required here, already initialised in timerInit() in the first place
		IOConfigGPIOAF(IOGetByTag(timer->tag), IOCFG_AF_PP_PD, timer->alternateFunction);

		/* PWM Input Capture configuration */
		pwmICConfig(timer->tim, timer->channel, TIM_ICPolarity_Rising);
		
		/* Timer configuration */
//		timerConfigure4UserBtn(timer, PWM_TIMER_PERIOD, PWM_TIMER_MHZ);
		timerConfigure(timer, (uint16_t)PWM_TIMER_PERIOD, PWM_TIMER_MHZ);


		/* Timer CC/Overflow callback configuration */
		timerChCCHandlerInit(&port->edgeCb, pwmEdgeCallback);
		timerChOvrHandlerInit(&port->overflowCb, pwmOverflowCallback);
//		printf("pwmEdgeCallback address: 0x%x, %s, %d\r\n", (uint32_t)pwmEdgeCallback, __FUNCTION__, __LINE__);				// 0x08001ba5
//		printf("pwmOverflowCallback address: 0x%x, %s, %d\r\n", (uint32_t)pwmOverflowCallback, __FUNCTION__, __LINE__);		// 0x08001c69
//		printf("port->edgeCb.fn address: 0x%x, %s, %d\r\n", (uint32_t)&port->edgeCb.fn, __FUNCTION__, __LINE__);			// 0x200017d8
//		printf("port->overflowCb.fn address: 0x%x, %s, %d\r\n", (uint32_t)&port->overflowCb.fn, __FUNCTION__, __LINE__);	// 0x200017dc
		timerChConfigCallbacks(timer, &port->edgeCb, &port->overflowCb);

//		TIM_ITConfig(timer->tim, TIM_IT_CC1, ENABLE);			// ENABLE TIM_IT_CC1 timer compare interrupt here
//		TIM_ITConfig(timer->tim, TIM_IT_Update, ENABLE);
	}
#else
//	TIM_ICInitTypeDef TIM5_ICInitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);		// Enable TIM5 clock
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);		// Enable PORTA clock
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;					// GPIO A0
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				// Alternate function
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			// Speed 100 MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				// Push-pull alternate output
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;				// Push down
//	GPIO_Init(GPIOA, &GPIO_InitStructure);						// Initialise GPIO A0
//	
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);		// Setup PA0 alternate function to TIM5
//	
//	/* Initialise TIME base configuration */
//	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / timerClockDivisor(TIM5) / ((uint32_t)PWM_TIMER_MHZ * 1000000)) - 1; // prescaler of timer
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	// Up mode
//	TIM_TimeBaseStructure.TIM_Period = PWM_TIMER_PERIOD;						// auto-reload value
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
//	
//	/* Initialise TIM5 Input Capture Parameters */
//	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1;			// CC1S = 01, mapping input IC1 to TI1
//	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;// Input capture occurs at rising edge of the signal
//	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	// map to TI1
//	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;		// No input divider
//	TIM5_ICInitStructure.TIM_ICFilter = 0x00;					// No input filter
//	TIM_ICInit(TIM5, &TIM5_ICInitStructure);
//	
//	/* Enable update interrupt, which allows CC1IE capturing interrupt */
////	TIM_ITConfig(TIM5, TIM_IT_Update | TIM_IT_CC1, ENABLE);
//	
//	/* Enable TIMER5 */
//	TIM_Cmd(TIM5, ENABLE);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	// Setup the preemption priority to 3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				// Enable IRQ channel
//	NVIC_Init(&NVIC_InitStructure);								// Initialise NVIC according to above parameters
//	
//	/* Enable update interrupt, which allows CC1IE capturing interrupt */
//	TIM_ITConfig(TIM5, TIM_IT_Update | TIM_IT_CC1, ENABLE);
#endif
}

#if 1
uint16_t pwmRead(uint8_t channel)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	return captures[channel];
}
#else
// TODO: change back to return type uint16_t later
uint32_t pwmRead(uint8_t channel)
{
	return captures[channel];
}
#endif

bool isPWMDataBeingReceived(void)
{
	int channel;
	for (channel = 0; channel < PWM_PORTS_OR_PPM_CAPTURE_COUNT; channel++) {	// PWM_PORTS_OR_PPM_CAPTURE_COUNT = 8
		if (captures[channel] != PPM_RCVR_TIMEOUT) {
			return true;
		}
	}
	
	return false;
}

