#ifndef __SERIAL_UART_IMPL_H
#define __SERIAL_UART_IMPL_H

#include <stdint.h>
#include "serial.h"
#include "serial_uart.h"
#include "target.h"

extern const struct serialPortVTable uartVTable[];

uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUART6(uint32_t baudRate, portMode_t mode, portOptions_t options);

#endif	// __SERIAL_UART_IMPL_H
