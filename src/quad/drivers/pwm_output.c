
#include <stdio.h>
#include <string.h>
#include "pwm_output.h"			// including timer.h, ledTimer.h, motors.h
#include "timer.h"

static pwmWriteFuncPtr pwmWritePtr;
static pwmOutputPort_t timerLeds[LED_NUMBER];
static pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmCompleteWriteFuncPtr pwmCompleteWritePtr = NULL;

bool pwmMotorsEnabled = false;

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	
	if (output & TIMER_OUTPUT_N_CHANNEL) {
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
		TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_High : TIM_OCNPolarity_Low;
	}else {
//		printf("output: %u, %s, %d\r\n", output, __FUNCTION__, __LINE__);			// output = 1
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;	// TIM_OCInitStructure.TIM_OCPolarity = 0x0
//		printf("TIM_OCInitStructure.TIM_OCPolarity: 0x%x, %s, %d\r\n", TIM_OCInitStructure.TIM_OCPolarity, __FUNCTION__, __LINE__);
	}
	
	TIM_OCInitStructure.TIM_Pulse = value;
	
	timerOCInit(tim, channel, &TIM_OCInitStructure);
	timerOCPreloadConfig(tim, channel, TIM_OCPreload_Enable);
}

/* value: idlePulse */
static void pwmOutConfigKhz(pwmOutputPort_t *port, const timerHardware_t *timerHardware, uint8_t khz, uint16_t period, uint16_t value)
{
    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
	
	/* Time base structure configuration */
	configTimeBaseKhz(timerHardware->tim, period, khz);		// khz = 10， period = 10 * 500 = 5000
	
	/* Output compare PWM generator configuration */
	pwmOCConfig(timerHardware->tim, timerHardware->channel, value, timerHardware->output);
	
	TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);		// for advanced TIMER, TIM1 and TIM8 to output PWM signals
	TIM_Cmd(timerHardware->tim, ENABLE);				// Enable TIMER
	
	port->ccr = timerChCCR(timerHardware);				// Config the channel address for duty cycle
//	printf("port->ccr: 0x%x, %s, %d\r\n", (uint32_t)port->ccr, __FUNCTION__, __LINE__);
	port->period = period;								// Config the PWM period
	port->tim = timerHardware->tim;						// Config the Timer number
	
	*port->ccr = 0;										// No duty cycle
}

/* value: idlePulse */
static void pwmOutConfigMhz(pwmOutputPort_t *port, const timerHardware_t *timerHardware, uint8_t mhz, uint16_t period, uint16_t value)
{
	/* Time base structure configuration */
	configTimeBaseMhz(timerHardware->tim, period, mhz);
	
	/* Output compare PWM generator configuration */
	pwmOCConfig(timerHardware->tim, timerHardware->channel, value, timerHardware->output);
	
	TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);		// for advanced TIMER, TIM1 and TIM8 to output PWM signals
	TIM_Cmd(timerHardware->tim, ENABLE);				// Enable TIMER
	
	port->ccr = timerChCCR(timerHardware);				// Config the channel address for duty cycle
//	printf("port->ccr: 0x%x, %s, %d\r\n", (uint32_t)port->ccr, __FUNCTION__, __LINE__);
//	printf("period: %u\r\n", period);
	port->period = period;								// Config the PWM period
//	printf("port->period: %u\r\n", port->period);
	port->tim = timerHardware->tim;						// Config the Timer number
	
	*port->ccr = 0;										// No duty cycle
}

static void pwmWriteStandard(uint8_t index, uint16_t value)
{
//	printf("value: %u\r\n", value);
	*motors[index].ccr = value;
}

static void pwmWriteUnused(uint8_t index, uint16_t value)
{
	UNUSED(index);
	UNUSED(value);
}

void pwmWriteLed(uint8_t index, uint16_t value)
{
	*timerLeds[index].ccr = value;
}

void pwmWriteMotor(uint8_t index, uint16_t value)
{
//	*motors[index].ccr = value;
//	printf("value: %u\r\n", value);
	pwmWritePtr(index, value);
}

bool pwmAreMotorsEnabled(void)
{
	return pwmMotorsEnabled;
}

void pwmCompleteMotorUpdate(uint8_t motorCount)
{
	pwmCompleteWritePtr(motorCount);
}

static void pwmCompleteWriteUnused(uint8_t motorCount)
{
	UNUSED(motorCount);
}

static void pwmCompleteOneshotMotorUpdate(uint8_t motorCount)
{
	for (int index = 0; index < motorCount; index++) {
		if (motors[index].forceOverflow) {
			timerForceOverflow(motors[index].tim);
		}
		
		/* Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again. 
		 * This compare register will be set to the output value on the next main loop
		 */
		*motors[index].ccr = 0;
	}
}

void motorInit(const motorConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount)
{
	memset(motors, 0, sizeof(motors));
	
	uint32_t timerMhzCounter = 0;
	bool useUnsyncedPwm = motorConfig->useUnsyncedPwm;			// false by default settings in ResetMotorConfig() function
	bool isDigital = false;
	
	switch (motorConfig->motorPwmProtocol) {
		default:
		case PWM_TYPE_ONESHOT125:		// maximum potential PID looptime is 2K microseconds depends on FC
			break;
		case PWM_TYPE_ONESHOT42:		// maximum potential PID looptime is 4K microseconds depends on FC
			break;
		case PWM_TYPE_MULTISHOT:		// maximum potential PID looptime is 8K microseconds depends on FC
			break;
		case PWM_TYPE_BRUSHED:
			break;
		case PWM_TYPE_STANDARD:			// maximum potential PID looptime is 0.5K(500) microseconds
			timerMhzCounter = PWM_TIMER_MHZ;
			pwmWritePtr = pwmWriteStandard;
//			printf("PWM_TYPE_STANDARD, %s, %d\r\n", __FUNCTION__, __LINE__);
			useUnsyncedPwm = false;		// default to using synced PWM rate, which means the motors PWM signals are updated at the end of each PID looptime
//			useUnsyncedPwm = true;		// unsynced PWM rate means the motors PWM signals are updated separatly from the PID looptime
			idlePulse = 0;
			break;
	}
	
	if (!isDigital) {
		pwmCompleteWritePtr = useUnsyncedPwm ? pwmCompleteWriteUnused : pwmCompleteOneshotMotorUpdate;
	}
	
//	printf("motorCount: %u, %s, %d\r\n", motorCount, __FUNCTION__, __LINE__);		// motorCount = 4
	
	for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
		const ioTag_t tag = motorConfig->ioTags[motorIndex];
		const timerHardware_t *timerHardware = timerGetByTag(tag, TIM_USE_ANY);
		
		if (timerHardware == NULL) {
//			printf("timerHardware == NULL, %s, %d\r\n", __FUNCTION__, __LINE__);
			pwmWritePtr = pwmWriteUnused;
			pwmCompleteWritePtr = pwmCompleteWriteUnused;
			/* TODO: block arming and add reason system cannot arm */
			return;
		}
		
//		printf("tag: 0x%x\r\n", tag);
		motors[motorIndex].io = IOGetByTag(tag);
		/* Tested successfully */
//		printf("IO_GPIO(motors[motorIndex].io): 0x%x\r\n", (uint32_t)IO_GPIO(motors[motorIndex].io));		// gpio = 0x40020C00 (GPIOD)
//		printf("IO_Pin(motors[motorIndex].io): %u\r\n\r\n", IO_Pin(motors[motorIndex].io));				// pin = (1<<12 | 1<<13 | 1<<14 | 1<<15)
		
#ifdef USE_DSHOT
		/* TODO: add digital motor (DSHOT) configuration function */
		
#endif		
		
		IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
		IOConfigGPIO(motors[motorIndex].io, IOCFG_AF_PP);
		
		/* Timer PWM generator configuration */
		if (useUnsyncedPwm) {
			/* motors pwm signals get updated individually, which are separated from PID looptime */
//			printf("unsynced, %s, %d\r\n", __FUNCTION__, __LINE__);
//			printf("timerHardware->tim: 0x%x, %s, %d\r\n", (uint32_t)timerHardware->tim, __FUNCTION__, __LINE__);
//			printf("timerHardware->tag: 0x%x, %s, %d\r\n", timerHardware->tag, __FUNCTION__, __LINE__);
//			printf("timerHardware->channel: %u, %s, %d\r\n", timerHardware->channel, __FUNCTION__, __LINE__);
//			printf("timerHardware->usageFlags: 0x%x, %s, %d\r\n", timerHardware->usageFlags, __FUNCTION__, __LINE__);
//			printf("timerHardware->alternateFunction: %u, %s, %d\r\n", timerHardware->alternateFunction, __FUNCTION__, __LINE__);
			const uint32_t periodHz = timerMhzCounter * 1000000;		// periodHz(PWM signal) = timerMhzCounter * 1000000 = 1 * 1000000 = 1000000 results in 1 MHz PWM frequency
//			const uint32_t periodHz = 1000;							// periodHz is TIMx_ARR(重装载值), PWM frequency = 1000000 / 500 = 2000 Hz
//			printf("periodHz: %u, %s, %d\r\n", periodHz, __FUNCTION__, __LINE__);		// motorCount = 4
			pwmOutConfigMhz(&motors[motorIndex], timerHardware, timerMhzCounter, periodHz / motorConfig->motorPwmRate, idlePulse);		// idlePulse = 0
//			pwmOutConfigMhz(&motors[motorIndex], timerHardware, timerMhzCounter, periodHz / motorConfig->motorPwmRate, idlePulse);
		}else {
			/* motors pwm signals get updated and synced at the end of each PID looptime */
//			printf("synced, %s, %d\r\n", __FUNCTION__, __LINE__);
			pwmOutConfigMhz(&motors[motorIndex], timerHardware, timerMhzCounter, 0xFFFF, 0);		// period = 0xFFFF (65535), idlePulse = 0
//			pwmOutConfigMhz(&timerLeds[motorIndex], timerHardware, timerMhzCounter, 0xFFFF, 0);		// period = 0xFFFF, idlePulse = 0
		}

		bool timerAlreadyUsed = false;
		for (int i = 0; i < motorIndex; i++) {
			if (motors[i].tim == motors[motorIndex].tim) {
				timerAlreadyUsed = true;
				break;
			}
		}
		motors[motorIndex].forceOverflow = !timerAlreadyUsed;
		
		motors[motorIndex].enabled = true;
	}
	
	/* ledTimerConfig->ioTags[xxx] initialisation done correctly */
//	printf("ledTimerConfig->ioTags[0]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[0], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
//	printf("ledTimerConfig->ioTags[1]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[1], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
//	printf("ledTimerConfig->ioTags[2]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[2], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
//	printf("ledTimerConfig->ioTags[3]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[3], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
	pwmMotorsEnabled = true;
}

#if 1
void ledTimerMhzInit(const LedTimerConfig_t *ledTimerConfig, uint16_t idlePulse, uint8_t ledCount)
{
	memset(timerLeds, 0, sizeof(timerLeds));
	
	uint32_t timerMhzCounter = PWM_TIMER_MHZ;
//	pwmWritePtr = pwmWriteLed;
	
	for (int ledIndex = 0; ledIndex < ledCount; ledIndex++) {
		const ioTag_t tag = ledTimerConfig->ioTags[ledIndex];
		const timerHardware_t *timerHardware = timerGetByTag(tag, TIM_USE_ANY);
		
		if (timerHardware == NULL) {
//			printf("timerHardware == NULL, %s, %d\r\n", __FUNCTION__, __LINE__);
			pwmWritePtr = pwmWriteUnused;
			return;
		}
		
		timerLeds[ledIndex].io = IOGetByTag(tag);
		/* Tested successfully */
//		printf("IO_GPIO(timerLeds[ledIndex].io): 0x%x\r\n", (uint32_t)IO_GPIO(timerLeds[ledIndex].io));		// gpio = 0x40020C00 (GPIOD)
//		printf("IO_Pin(timerLeds[ledIndex].io): %u\r\n\r\n", IO_Pin(timerLeds[ledIndex].io));				// pin = (1<<12 | 1<<13 | 1<<14 | 1<<15)
		
		IOInit(timerLeds[ledIndex].io, OWNER_TIMER_LED, RESOURCE_INDEX(ledIndex));
		IOConfigGPIO(timerLeds[ledIndex].io, IOCFG_AF_PP);
		
		/* Timer PWM generator configuration */
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//		printf("timerHardware->tim: 0x%x, %s, %d\r\n", (uint32_t)timerHardware->tim, __FUNCTION__, __LINE__);
//		printf("timerHardware->tag: 0x%x, %s, %d\r\n", timerHardware->tag, __FUNCTION__, __LINE__);
//		printf("timerHardware->channel: %u, %s, %d\r\n", timerHardware->channel, __FUNCTION__, __LINE__);
//		printf("timerHardware->usageFlags: 0x%x, %s, %d\r\n", timerHardware->usageFlags, __FUNCTION__, __LINE__);
//		printf("timerHardware->alternateFunction: %u, %s, %d\r\n", timerHardware->alternateFunction, __FUNCTION__, __LINE__);
//		const uint32_t periodHz = timerKhzCounter * KhzGain;		// KhzGain = 1000, periodHz(PWM signal) = timerKhzCounter * KhzGain = 10 * 1000 = 10000 results in 1 Hz PWM frequency (timerKhzCounter * 1000 / periodHz = 10 * 1000 / 10000 = 1 Hz)
//		const uint32_t periodHz = timerKhzCounter * 500;		// periodHz(PWM signal) = timerKhzCounter * 500 = 10 * 500 = 5000 results in 2 Hz PWM frequency (timerKhzCounter * 1000 / periodHz = 10 * 1000 / 5000 = 2 Hz)
//		const uint32_t periodHz = 0xFFFF;							// periodHz is TIMx_ARR(重装载值), PWM frequency = 1000000 / 500 = 2000 Hz
		const uint32_t periodHz = 1000;							// periodHz is TIMx_ARR(重装载值), PWM frequency = 1000000 / 500 = 2000 Hz
//		printf("periodHz: %u\r\n", periodHz);
		pwmOutConfigMhz(&timerLeds[ledIndex], timerHardware, timerMhzCounter, periodHz, idlePulse);
		
		timerLeds[ledIndex].enabled = true;
	}
	
	/* ledTimerConfig->ioTags[xxx] initialisation done correctly */
//	printf("ledTimerConfig->ioTags[0]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[0], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
//	printf("ledTimerConfig->ioTags[1]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[1], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
//	printf("ledTimerConfig->ioTags[2]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[2], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
//	printf("ledTimerConfig->ioTags[3]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[3], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
}
#else
/* KhzGain = 1000 (PWM frequency 1 Hz) or 500 (PWM frequency 2 Hz) in this case to light up LEDs */
void ledTimerKhzInit(const LedTimerConfig_t *ledTimerConfig, uint16_t idlePulse, uint8_t ledCount, uint32_t KhzGain)
{
	memset(timerLeds, 0, sizeof(timerLeds));
	
	uint32_t timerKhzCounter = PWM_TIMER_KHZ;
//	pwmWritePtr = pwmWriteStandard;
	
	for (int ledIndex = 0; ledIndex < ledCount; ledIndex++) {
		const ioTag_t tag = ledTimerConfig->ioTags[ledIndex];
		const timerHardware_t *timerHardware = timerGetByTag(tag, TIM_USE_ANY);
		
		if (timerHardware == NULL) {
			pwmWritePtr = pwmWriteUnused;
			return;
		}
		
		timerLeds[ledIndex].io = IOGetByTag(tag);
		/* Tested successfully */
//		printf("IO_GPIO(timerLeds[ledIndex].io): 0x%x\r\n", (uint32_t)IO_GPIO(timerLeds[ledIndex].io));		// gpio = 0x40020C00 (GPIOD)
//		printf("IO_Pin(timerLeds[ledIndex].io): %u\r\n\r\n", IO_Pin(timerLeds[ledIndex].io));				// pin = (1<<12 | 1<<13 | 1<<14 | 1<<15)
		
		IOInit(timerLeds[ledIndex].io, OWNER_TIMER_LED, RESOURCE_INDEX(ledIndex));
		IOConfigGPIO(timerLeds[ledIndex].io, IOCFG_AF_PP);
		
		/* Timer PWM generator configuration */
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//		printf("timerHardware->tim: 0x%x, %s, %d\r\n", (uint32_t)timerHardware->tim, __FUNCTION__, __LINE__);
//		printf("timerHardware->tag: 0x%x, %s, %d\r\n", timerHardware->tag, __FUNCTION__, __LINE__);
//		printf("timerHardware->channel: %u, %s, %d\r\n", timerHardware->channel, __FUNCTION__, __LINE__);
//		printf("timerHardware->usageFlags: 0x%x, %s, %d\r\n", timerHardware->usageFlags, __FUNCTION__, __LINE__);
//		printf("timerHardware->alternateFunction: %u, %s, %d\r\n", timerHardware->alternateFunction, __FUNCTION__, __LINE__);
		const uint32_t periodHz = timerKhzCounter * KhzGain;		// KhzGain = 1000, periodHz(PWM signal) = timerKhzCounter * KhzGain = 10 * 1000 = 10000 results in 1 Hz PWM frequency (timerKhzCounter * 1000 / periodHz = 10 * 1000 / 10000 = 1 Hz)
//		const uint32_t periodHz = timerKhzCounter * 500;		// periodHz(PWM signal) = timerKhzCounter * 500 = 10 * 500 = 5000 results in 2 Hz PWM frequency (timerKhzCounter * 1000 / periodHz = 10 * 1000 / 5000 = 2 Hz)
//		printf("periodHz: %u\r\n", periodHz);
		pwmOutConfigKhz(&timerLeds[ledIndex], timerHardware, timerKhzCounter, periodHz, idlePulse);
//		pwmOutConfigKhz(&timerLeds[ledIndex], timerHardware, timerKhzCounter, 0xFFFF, 0);		// period = 0xFFFF, idlePulse = 0
		
		timerLeds[ledIndex].enabled = true;
	}
	
	/* ledTimerConfig->ioTags[xxx] initialisation done correctly */
//	printf("ledTimerConfig->ioTags[0]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[0], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
//	printf("ledTimerConfig->ioTags[1]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[1], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
//	printf("ledTimerConfig->ioTags[2]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[2], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
//	printf("ledTimerConfig->ioTags[3]: 0x%x, %s, %d\r\n", ledTimerConfig->ioTags[3], __FUNCTION__, __LINE__);	// LED4, PD12, ioTag_t 0x4C
}
#endif
