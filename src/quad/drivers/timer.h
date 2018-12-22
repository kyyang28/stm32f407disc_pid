#ifndef __TIMER_H
#define __TIMER_H

#include "io.h"
#include "rcc.h"

// platform.h includes target.h, stm32f4xx_tim.h, stm32f4xx_gpio.h, stm32f4xx_dma.h
#include "platform.h"

typedef uint16_t captureCompare_t;        // 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)

#if defined(STM32F4)
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;
#else
#error "Unknown CPU defined"
#endif

typedef enum {
    TIM_USE_ANY           = 0x0,
    TIM_USE_NONE          = 0x0,
    TIM_USE_PPM           = 0x1,
    TIM_USE_PWM           = 0x2,
    TIM_USE_MOTOR         = 0x4,
    TIM_USE_SERVO         = 0x8,
    TIM_USE_LED           = 0x10,
    TIM_USE_TRANSPONDER   = 0x20,
    TIM_USE_BEEPER        = 0x40
} timerUsageFlag_e;

// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
struct timerCCHandlerRec_s;
struct timerOvrHandlerRec_s;
typedef void timerCCHandlerCallback(struct timerCCHandlerRec_s* self, uint32_t capture);		// TODO: need to figure out why betaflight uses uint16_t capture
typedef void timerOvrHandlerCallback(struct timerOvrHandlerRec_s* self, uint32_t capture);		// TODO: stick with uint32_t capture FOR NOW
//typedef void timerCCHandlerCallback(struct timerCCHandlerRec_s* self, uint16_t capture);
//typedef void timerOvrHandlerCallback(struct timerOvrHandlerRec_s* self, uint16_t capture);

typedef struct timerCCHandlerRec_s {
    timerCCHandlerCallback* fn;
} timerCCHandlerRec_t;

typedef struct timerOvrHandlerRec_s {
    timerOvrHandlerCallback* fn;
    struct timerOvrHandlerRec_s* next;
} timerOvrHandlerRec_t;

typedef struct timerDef_s {
    TIM_TypeDef *TIMx;
    RccPeriphTag_t rcc;
    uint8_t inputIrq;
} timerDef_t;

typedef struct timerHardware_s {
    TIM_TypeDef *tim;
    ioTag_t tag;
    uint8_t channel;
    timerUsageFlag_e usageFlags;
    uint8_t output;
#if defined(STM32F4)
    uint8_t alternateFunction;
#endif
#if defined(USE_DSHOT) || defined(USE_LED_STRIP) || defined(USE_TRANSPONDER)
#if defined(STM32F4)
    DMA_Stream_TypeDef *dmaStream;
    uint32_t dmaChannel;
#elif defined(STM32F3) || defined(STM32F1)
	DMA_Channel_TypeDef *dmaChannel;
#endif
    uint8_t dmaIrqHandler;
#endif
} timerHardware_t;

typedef enum {
    TIMER_OUTPUT_NONE      = 0x00,
    TIMER_INPUT_ENABLED    = 0x01, /* TODO: remove this */
    TIMER_OUTPUT_ENABLED   = 0x01, /* TODO: remove this */
    TIMER_OUTPUT_STANDARD  = 0x01,
    TIMER_OUTPUT_INVERTED  = 0x02,
    TIMER_OUTPUT_N_CHANNEL = 0x04
} timerFlag_e;

#define HARDWARE_TIMER_DEFINITION_COUNT 		14

extern const timerHardware_t timerHardware[];
extern const timerDef_t timerDefinitions[];

typedef enum {
    TYPE_FREE,
    TYPE_PWMINPUT,
    TYPE_PPMINPUT,
    TYPE_PWMOUTPUT_MOTOR,
    TYPE_PWMOUTPUT_FAST,
    TYPE_PWMOUTPUT_SERVO,
    TYPE_SOFTSERIAL_RX,
    TYPE_SOFTSERIAL_TX,
    TYPE_SOFTSERIAL_RXTX,        // bidirectional pin for softserial
    TYPE_SOFTSERIAL_AUXTIMER,    // timer channel is used for softserial. No IO function on pin
    TYPE_ADC,
    TYPE_SERIAL_RX,
    TYPE_SERIAL_TX,
    TYPE_SERIAL_RXTX,
    TYPE_TIMER
} channelType_t;

void timerInit(void);
const timerHardware_t *timerGetByTag(ioTag_t tag, timerUsageFlag_e flag);
void configTimeBaseKhz(TIM_TypeDef *tim, uint16_t period, uint8_t khz);
void configTimeBaseMhz(TIM_TypeDef *tim, uint16_t period, uint8_t Mhz);
void configTimeBaseMhz4UserBtn(TIM_TypeDef *tim, uint32_t period, uint8_t Mhz);

uint8_t timerClockDivisor(TIM_TypeDef *tim);
void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init);
void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload);

volatile timCCR_t* timerChCCR(const timerHardware_t *timHw);

void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz);
void timerConfigure4UserBtn(const timerHardware_t *timerHardwarePtr, uint32_t period, uint8_t mhz);

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChConfigCallbacks(const timerHardware_t *timHw, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);

void timerForceOverflow(TIM_TypeDef *tim);

#endif	// __TIMER_H
