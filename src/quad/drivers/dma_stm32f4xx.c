
#include <stdio.h>				// debugging purposes
#include "platform.h"
#include "dma.h"

/*
 * DMA descriptors
 * #define DEFINE_DMA_CHANNEL(d, s, f, i, r)
 *		{.dma = d, .stream = s, .irqHandlerCallback = NULL, .flagsShift = f, .irqN = i, .rcc = r, .userParam = 0, .owner = 0, .resourceIndex = 0}
 */
static dmaChannelDescriptor_t dmaDescriptors[DMA_MAX_DESCRIPTORS] = {
	DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream0, 0,  DMA1_Stream0_IRQn, RCC_AHB1Periph_DMA1),
	DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream1, 6,  DMA1_Stream1_IRQn, RCC_AHB1Periph_DMA1),
	DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream2, 16, DMA1_Stream2_IRQn, RCC_AHB1Periph_DMA1),
	DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream3, 22, DMA1_Stream3_IRQn, RCC_AHB1Periph_DMA1),
	DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream4, 32, DMA1_Stream4_IRQn, RCC_AHB1Periph_DMA1),
	DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream5, 38, DMA1_Stream5_IRQn, RCC_AHB1Periph_DMA1),
	DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream6, 48, DMA1_Stream6_IRQn, RCC_AHB1Periph_DMA1),
	DEFINE_DMA_CHANNEL(DMA1, DMA1_Stream7, 54, DMA1_Stream7_IRQn, RCC_AHB1Periph_DMA1),

	DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream0, 0,  DMA2_Stream0_IRQn, RCC_AHB1Periph_DMA2),
	DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream1, 6,  DMA2_Stream1_IRQn, RCC_AHB1Periph_DMA2),
	DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream2, 16, DMA2_Stream2_IRQn, RCC_AHB1Periph_DMA2),
	DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream3, 22, DMA2_Stream3_IRQn, RCC_AHB1Periph_DMA2),
	DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream4, 32, DMA2_Stream4_IRQn, RCC_AHB1Periph_DMA2),
	DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream5, 38, DMA2_Stream5_IRQn, RCC_AHB1Periph_DMA2),
	DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream6, 48, DMA2_Stream6_IRQn, RCC_AHB1Periph_DMA2),
	DEFINE_DMA_CHANNEL(DMA2, DMA2_Stream7, 54, DMA2_Stream7_IRQn, RCC_AHB1Periph_DMA2),
};

/**
 * DMA IRQ Handlers
 */
DEFINE_DMA_IRQ_HANDLER(1, 0, DMA1_ST0_HANDLER)		// 1 means DMA1, 0 means Stream0, DMA1_ST0_HANDLER = 0
DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_ST1_HANDLER)		// 1 means DMA1, 1 means Stream1, DMA1_ST1_HANDLER = 1
DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1_ST2_HANDLER)		// 1 means DMA1, 2 means Stream2, DMA1_ST2_HANDLER = 2
DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1_ST3_HANDLER)		// 1 means DMA1, 3 means Stream3, DMA1_ST3_HANDLER = 3
DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1_ST4_HANDLER)		// 1 means DMA1, 4 means Stream4, DMA1_ST4_HANDLER = 4
DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1_ST5_HANDLER)		// 1 means DMA1, 5 means Stream5, DMA1_ST5_HANDLER = 5
DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1_ST6_HANDLER)		// 1 means DMA1, 6 means Stream6, DMA1_ST6_HANDLER = 6
DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1_ST7_HANDLER)		// 1 means DMA1, 7 means Stream7, DMA1_ST7_HANDLER = 7

DEFINE_DMA_IRQ_HANDLER(2, 0, DMA2_ST0_HANDLER)		// 2 means DMA2, 0 means Stream0, DMA2_ST0_HANDLER = 8
DEFINE_DMA_IRQ_HANDLER(2, 1, DMA2_ST1_HANDLER)		// 2 means DMA2, 1 means Stream1, DMA2_ST1_HANDLER = 9
DEFINE_DMA_IRQ_HANDLER(2, 2, DMA2_ST2_HANDLER)		// 2 means DMA2, 2 means Stream2, DMA2_ST2_HANDLER = 10
DEFINE_DMA_IRQ_HANDLER(2, 3, DMA2_ST3_HANDLER)		// 2 means DMA2, 3 means Stream3, DMA2_ST3_HANDLER = 11
DEFINE_DMA_IRQ_HANDLER(2, 4, DMA2_ST4_HANDLER)		// 2 means DMA2, 4 means Stream4, DMA2_ST4_HANDLER = 12
DEFINE_DMA_IRQ_HANDLER(2, 5, DMA2_ST5_HANDLER)		// 2 means DMA2, 5 means Stream5, DMA2_ST5_HANDLER = 13
DEFINE_DMA_IRQ_HANDLER(2, 6, DMA2_ST6_HANDLER)		// 2 means DMA2, 6 means Stream6, DMA2_ST6_HANDLER = 14
DEFINE_DMA_IRQ_HANDLER(2, 7, DMA2_ST7_HANDLER)		// 2 means DMA2, 7 means Stream7, DMA2_ST7_HANDLER = 15

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef *stream)
{
	for (int i = 0; i < DMA_MAX_DESCRIPTORS; i++) {
		if (dmaDescriptors[i].stream == stream) {
			return i;
		}
	}
	
	return 0;
}

void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex)
{
//	printf("identifier: %d, %s, %d\r\n", identifier, __FUNCTION__, __LINE__);
	RCC_AHB1PeriphClockCmd(dmaDescriptors[identifier].rcc, ENABLE);
	dmaDescriptors[identifier].owner = owner;
	dmaDescriptors[identifier].resourceIndex = resourceIndex;
}
