
#include "timer.h"
#include "dma.h"

//typedef struct timerHardware_s {
//    TIM_TypeDef *tim;
//    ioTag_t tag;
//    uint8_t channel;
//    timerUsageFlag_e usageFlags;
//    uint8_t output;
//    uint8_t alternateFunction;
//    DMA_Stream_TypeDef *dmaStream;
//    uint32_t dmaChannel;
//    uint8_t dmaIrqHandler;
//} timerHardware_t;

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	{	// PWM IN and LED4 OUT
		.tim = TIM4,
		.tag = IO_TAG(LED4),						// GREEN LED (LED4)	PD12, ioTag_t: DEFIO_TAG__PD12 (0x4C)
//		.tag = IO_TAG(PB9),
		.channel = TIM_Channel_1,
		.usageFlags = TIM_USE_LED,
//		.usageFlags = TIM_USE_PWM | TIM_USE_LED,
//		.usageFlags = TIM_USE_PPM | TIM_USE_LED,
		.output = TIMER_OUTPUT_NONE | 1,
		.alternateFunction = GPIO_AF_TIM4,
		.dmaStream = NULL,
		.dmaChannel = 0,
		.dmaIrqHandler = 0
	},
	{	// PWM IN and LED3 OUT
		.tim = TIM4,
		.tag = IO_TAG(LED3),						// ORANGE LED (LED3) PD13, ioTag_t: DEFIO_TAG__PD13 (0x4D)
//		.tag = IO_TAG(PB9),
		.channel = TIM_Channel_2,
		.usageFlags = TIM_USE_LED,
//		.usageFlags = TIM_USE_PWM | TIM_USE_LED,
//		.usageFlags = TIM_USE_PPM | TIM_USE_LED,
		.output = TIMER_OUTPUT_NONE | 1,
		.alternateFunction = GPIO_AF_TIM4,
		.dmaStream = NULL,
		.dmaChannel = 0,
		.dmaIrqHandler = 0
	},
	{	// PWM IN and LED5 OUT
		.tim = TIM4,
		.tag = IO_TAG(LED5),						// RED LED (LED5) PD14, ioTag_t: DEFIO_TAG__PD14 (0x4E)
//		.tag = IO_TAG(PB9),
		.channel = TIM_Channel_3,
		.usageFlags = TIM_USE_LED,
//		.usageFlags = TIM_USE_PWM | TIM_USE_LED,
//		.usageFlags = TIM_USE_PPM | TIM_USE_LED,
		.output = TIMER_OUTPUT_NONE | 1,
		.alternateFunction = GPIO_AF_TIM4,
		.dmaStream = NULL,
		.dmaChannel = 0,
		.dmaIrqHandler = 0
	},
	{	// PWM IN and LED6 OUT
		.tim = TIM4,
		.tag = IO_TAG(LED6),						// BLUE LED (LED6) PD15, ioTag_t: DEFIO_TAG__PD15 (0x4F)
//		.tag = IO_TAG(PB9),
		.channel = TIM_Channel_4,
		.usageFlags = TIM_USE_LED,
//		.usageFlags = TIM_USE_PWM | TIM_USE_LED,
//		.usageFlags = TIM_USE_PPM | TIM_USE_LED,
		.output = TIMER_OUTPUT_NONE | 1,
		.alternateFunction = GPIO_AF_TIM4,
		.dmaStream = NULL,
		.dmaChannel = 0,
		.dmaIrqHandler = 0
	},
	{	// S1_OUT - TIM3_UP - BURST
		.tim = TIM5,
		.tag = IO_TAG(PA0),
		.channel = TIM_Channel_1,
		.usageFlags = TIM_USE_PWM,
		.output = TIMER_OUTPUT_NONE | 1,
		.alternateFunction = GPIO_AF_TIM5,
		.dmaStream = NULL,
		.dmaChannel = 0,
		.dmaIrqHandler = 0
	},
	{	// S1_OUT - TIM3_UP - BURST
		.tim = TIM5,
		.tag = IO_TAG(PA1),
		.channel = TIM_Channel_2,
		.usageFlags = TIM_USE_PWM,
		.output = TIMER_OUTPUT_NONE | 1,
		.alternateFunction = GPIO_AF_TIM5,
		.dmaStream = NULL,
		.dmaChannel = 0,
		.dmaIrqHandler = 0
	},
	{	// S1_OUT - TIM3_UP - BURST
		.tim = TIM3,
		.tag = IO_TAG(PB1),
		.channel = TIM_Channel_4,
		.usageFlags = TIM_USE_MOTOR,
		.output = TIMER_OUTPUT_NONE,
		.alternateFunction = GPIO_AF_TIM3,
		.dmaStream = DMA1_Stream2,
		.dmaChannel = DMA_Channel_5,
		.dmaIrqHandler = DMA1_ST2_HANDLER			// DMA1_ST2_HANDLER = 2
	},
};
