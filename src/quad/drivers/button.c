
#include <stdlib.h>
#include <string.h>
#include "io.h"
#include "led.h"
#include "button.h"
#include "exti.h"
#include "nvic.h"
#include "target.h"
//#include "stdio.h"

IO_t userBtnPin;
bool buttonPressed;

button_t button;

static const extiConfig_t *selectBtnIntExtiConfig(void)
{
#if defined(BTN_INT_EXTI)
	static const extiConfig_t btnIntExtiConfig = { .tag = IO_TAG(USER_BUTTON_PIN) };	// tag = IO_TAG(USER_BUTTON_PIN) = 0x10
	return &btnIntExtiConfig;
#else
	return NULL;
#endif
}

#if defined(BTN_INT_EXTI)
static void btnIntExtiHandler(extiCallbackRec_t *cb)
{
//	LED6_ON;
	button.dev.btnPressed = true;
	LED3_TOGGLE;
	LED4_TOGGLE;
	LED5_TOGGLE;
	LED6_TOGGLE;
}
#endif

static bool btnIntExtiInit(buttonDev_t *dev)
{
#if defined(BTN_INT_EXTI)	
	if (!dev->btnIntExtiConfig) {
		return false;
	}
	
	IO_t btnIntIO = IOGetByTag(dev->btnIntExtiConfig->tag);		// dev->btnIntExtiConfig->tag = 0x10
	IOInit(btnIntIO, OWNER_BUTTON, 0);
	IOConfigGPIO(btnIntIO, IOCFG_IPD);		// btnIntIO gpio = portA, pin = 0
//	LED6_ON;
	
	/* EXTI */
	EXTIHandlerInit(&dev->exti, btnIntExtiHandler);		// initialise btnIntExtiHandler to extiCallbackRec_t exti
	/* void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, EXTITrigger_TypeDef trigger) */
	EXTIConfig(btnIntIO, &dev->exti, NVIC_PRIO_USRBTN_EXTI, EXTI_Trigger_Falling);
	EXTIEnable(btnIntIO, true);
//	LED5_ON;
#endif
	
	return true;
}

bool buttonInit(void)
{
	memset(&button, 0, sizeof(button));
	button.dev.btnIntExtiConfig = selectBtnIntExtiConfig();		// button.dev.btnIntExtiConfig = 0x10
	if (!btnIntExtiInit(&button.dev)) {
		return false;
	}
	
	return true;
}

void userBtnPollInit(void)
{
	userBtnPin = IOGetByTag(IO_TAG(USER_BUTTON_PIN));
	IOInit(userBtnPin, OWNER_SYSTEM, 0);
	IOConfigGPIO(userBtnPin, IOCFG_IPD);
}

void userBtnPollOps(void)
{
	buttonPressed = IORead(userBtnPin);
//	printf("buttonPressed: %d\r\n", buttonPressed);
	if (buttonPressed) {
		LED3_ON;
		LED4_ON;
		LED5_ON;
		LED6_ON;
	}else {
		LED3_OFF;
		LED4_OFF;
		LED5_OFF;
		LED6_OFF;
	}
}
