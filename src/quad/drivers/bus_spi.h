#ifndef __BUS_SPI_H
#define __BUS_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "io.h"
#include "RCCTypes.h"
#include "platform.h"			// define STM32F4

#if defined(STM32F4)
#define SPI_IO_AF_CFG			IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_AF_SCK_CFG		IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
//#define SPI_IO_AF_SCK_CFG		IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define SPI_IO_AF_MISO_CFG		IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_AF_MOSI_CFG		IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG			IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#endif

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,			// 1
    SPIDEV_3,			// 2
    SPIDEV_4			// 3
} SPIDevice;

typedef struct SPIDevice_s {
    SPI_TypeDef *dev;
    ioTag_t nss;
    ioTag_t sck;
    ioTag_t mosi;
    ioTag_t miso;
    RccPeriphTag_t rcc;
    uint8_t af;
    volatile uint16_t errorCount;
    bool leadingEdge;
//#if defined(STM32F7)
//    SPI_HandleTypeDef hspi;
//    DMA_HandleTypeDef hdma;
//    uint8_t dmaIrqHandler;
//#endif
} spiDevice_t;

/*
  Flash M25p16 tolerates 20mhz, SPI_CLOCK_FAST should sit around 20 or less.
*/
typedef enum {
    SPI_CLOCK_INITIALISATION = 256,
#if defined(STM32F4)
    SPI_CLOCK_SLOW          = 128, //00.65625 MHz
    SPI_CLOCK_STANDARD      = 8,   //10.50000 MHz
    SPI_CLOCK_FAST          = 4,   //21.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //42.00000 MHz
#endif
} SPIClockDivider_e;

bool spiInit(SPIDevice device);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
bool spiIsBusBusy(SPI_TypeDef *instance);
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data);
bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len);
uint16_t spiGetErrorCounter(SPI_TypeDef *instance);
void spiResetErrorCounter(SPI_TypeDef *instance);

#endif	// __BUS_SPI_H
