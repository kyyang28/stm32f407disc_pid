#ifndef __DMA_H
#define __DMA_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"		// including DMA_TypeDef and DMA_Stream_TypeDef
#include "resource.h"

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
	DMA_TypeDef *dma;
	DMA_Stream_TypeDef *stream;
	dmaCallbackHandlerFuncPtr irqHandlerCallback;
	uint8_t flagsShift;
	IRQn_Type irqN;
	uint32_t rcc;
	uint32_t userParam;
	resourceOwner_e owner;
	uint8_t resourceIndex;
}dmaChannelDescriptor_t;

typedef enum {
    DMA1_ST0_HANDLER = 0,
    DMA1_ST1_HANDLER,			// 1
    DMA1_ST2_HANDLER,			// 2
    DMA1_ST3_HANDLER,			// 3
    DMA1_ST4_HANDLER,			// 4
    DMA1_ST5_HANDLER,			// 5
    DMA1_ST6_HANDLER,			// 6
    DMA1_ST7_HANDLER,			// 7
    DMA2_ST0_HANDLER,			// 8
    DMA2_ST1_HANDLER,			// 9
    DMA2_ST2_HANDLER,			// 10
    DMA2_ST3_HANDLER,			// 11
    DMA2_ST4_HANDLER,			// 12
    DMA2_ST5_HANDLER,			// 13
    DMA2_ST6_HANDLER,			// 14
    DMA2_ST7_HANDLER,			// 15
    DMA_MAX_DESCRIPTORS			// 16
} dmaIdentifier_e;

#define DEFINE_DMA_CHANNEL(d, s, f, i, r)	{.dma = d, .stream = s, .irqHandlerCallback = NULL, .flagsShift = f, .irqN = i, .rcc = r, .userParam = 0, .owner = 0, .resourceIndex = 0}
#define DEFINE_DMA_IRQ_HANDLER(d, s, i) void DMA ## d ## _Stream ## s ## _IRQHandler(void) {\
															if (dmaDescriptors[i].irqHandlerCallback)\
																dmaDescriptors[i].irqHandlerCallback(&dmaDescriptors[i]);\
														}\

dmaIdentifier_e dmaGetIdentifier(const DMA_Stream_TypeDef *stream);
void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex);

#endif	// __DMA_H
