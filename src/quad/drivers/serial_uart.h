#ifndef __SERIAL_UART_H
#define __SERIAL_UART_H

#include "stm32f4xx.h"
#include "serial.h"
#include "platform.h"
#include "target.h"

// Since serial ports can be used for any function these buffer sizes should be equal
// The two largest things that need to be sent are: 1, MSP responses, 2, UBLOX SVINFO packet.

// Size must be a power of two due to various optimizations which use 'and' instead of 'mod'
// Various serial routines return the buffer occupied size as uint8_t which would need to be extended in order to
// increase size further.
#define UART1_RX_BUFFER_SIZE    256
#define UART1_TX_BUFFER_SIZE    256
#define UART2_RX_BUFFER_SIZE    256
#define UART2_TX_BUFFER_SIZE    256
#define UART3_RX_BUFFER_SIZE    256
#define UART3_TX_BUFFER_SIZE    256
#define UART4_RX_BUFFER_SIZE    256
#define UART4_TX_BUFFER_SIZE    256
#define UART5_RX_BUFFER_SIZE    256
#define UART5_TX_BUFFER_SIZE    256
#define UART6_RX_BUFFER_SIZE    256
#define UART6_TX_BUFFER_SIZE    256
#define UART7_RX_BUFFER_SIZE    256
#define UART7_TX_BUFFER_SIZE    256
#define UART8_RX_BUFFER_SIZE    256
#define UART8_TX_BUFFER_SIZE    256

typedef struct {
    serialPort_t port;

#if defined(STM32F4)
    DMA_Stream_TypeDef *rxDMAStream;
    DMA_Stream_TypeDef *txDMAStream;
    uint32_t rxDMAChannel;
    uint32_t txDMAChannel;
#endif
    uint32_t rxDMAIrq;
    uint32_t txDMAIrq;

    uint32_t rxDMAPos;
    bool txDMAEmpty;

    uint32_t txDMAPeripheralBaseAddr;
    uint32_t rxDMAPeripheralBaseAddr;

#ifdef USE_HAL_DRIVER
    // All USARTs can also be used as UART, and we use them only as UART.
    UART_HandleTypeDef Handle;
#endif
    USART_TypeDef *USARTx;
} uartPort_t;

serialPort_t *uartOpen(USART_TypeDef *USARTx, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_t mode, portOptions_t options);

#endif	// __SERIAL_UART_H
