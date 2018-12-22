
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "exti.h"
#include "nvic.h"
#include "misc.h"
#include "target.h"

#define EXTI_IRQ_GROUPS					(7)

typedef struct {
	extiCallbackRec_t *handler;
}extiChannelRec_t;

extiChannelRec_t extiChannelRecs[16];
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];
static const uint8_t extiGroups[16] = { 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6 };

/* STM32F4 */
static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
	EXTI0_IRQn,
	EXTI1_IRQn,
	EXTI2_IRQn,
	EXTI3_IRQn,
	EXTI4_IRQn,
	EXTI9_5_IRQn,
	EXTI15_10_IRQn
};

void EXTIInit(void)
{
	/* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	memset(extiChannelRecs, 0, sizeof(extiChannelRecs));
	memset(extiGroupPriority, 0xff, sizeof(extiGroupPriority));
}

void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
	self->fn = fn;
}

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, EXTITrigger_TypeDef trigger)
{
	int channelIdx;
	channelIdx = IO_GPIOPinIdx(io);		// channelIdx = 0 (PA0)
	if (channelIdx < 0)
		return;
	
	extiChannelRec_t *rec = &extiChannelRecs[channelIdx];	// retieve extiCallbackRec_t exti from extiChannelRecs array
	int group = extiGroups[channelIdx];			// group = 0
	
	rec->handler = cb;
	
	/* Config EXTI with SYSCFG register */
	SYSCFG_EXTILineConfig(IO_EXTI_PortSourceGPIO(io), IO_EXTI_PinSource(io));
	
	uint32_t extiLine = IO_EXTI_Line(io);
	EXTI_ClearITPendingBit(extiLine);		// extiLine = 1 << 0 since PA0
	
	EXTI_InitTypeDef EXTIInit;
	EXTIInit.EXTI_Line		= extiLine;
	EXTIInit.EXTI_Mode		= EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger	= trigger;
	EXTIInit.EXTI_LineCmd	= ENABLE;
	EXTI_Init(&EXTIInit);
	
	if (extiGroupPriority[group] > irqPriority) {
		extiGroupPriority[group] = irqPriority;
		
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel						= extiGroupIRQn[group];
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= NVIC_PRIORITY_BASE(irqPriority);
		NVIC_InitStructure.NVIC_IRQChannelSubPriority			= NVIC_PRIORITY_SUB(irqPriority);
		NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}

void EXTIEnable(IO_t io, bool enable)
{
	uint32_t extiLine = IO_EXTI_Line(io);
	if (!extiLine)
		return;
	if (enable)
		EXTI->IMR |= extiLine;
	else
		EXTI->IMR &= ~extiLine;
}

void EXTIRelease(IO_t io)
{
	int chIdx;
	EXTIEnable(io, false);		// clear the extiLine
	
	chIdx = IO_GPIOPinIdx(io);
	if (chIdx < 0)
		return;
	extiChannelRec_t *rec = &extiChannelRecs[chIdx];
	rec->handler = NULL;
}

void EXTI_IRQHandler(void)
{
	uint32_t extiActive = EXTI->IMR & EXTI->PR;		// interrupt mask register is not masked and pending register contains occurred intterupt
	
	while (extiActive) {
		unsigned index = 31 - __builtin_clz(extiActive);		// __builtin_clz returns the number of leading zeros starting from the MSB
		uint32_t mask = 1 << index;
		extiChannelRecs[index].handler->fn(extiChannelRecs[index].handler);
		EXTI->PR = mask;			// clear pending mask by writing 1
		extiActive &= ~mask;
	}
}

#define __EXTI_IRQ_HANDLER(name)						\
	void name(void) {									\
		EXTI_IRQHandler();								\
	}													\
	//struct dummy

__EXTI_IRQ_HANDLER(EXTI0_IRQHandler);				// User button (PA0)
__EXTI_IRQ_HANDLER(EXTI1_IRQHandler);
__EXTI_IRQ_HANDLER(EXTI2_IRQHandler);
__EXTI_IRQ_HANDLER(EXTI3_IRQHandler);
__EXTI_IRQ_HANDLER(EXTI4_IRQHandler);
__EXTI_IRQ_HANDLER(EXTI9_5_IRQHandler);
__EXTI_IRQ_HANDLER(EXTI15_10_IRQHandler);
