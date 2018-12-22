#ifndef __PRINTF_H
#define __PRINTF_H

#include <stdarg.h>
#include "serial.h"

void init_printf(void *putp, void (*putf) (void *, char));

int tfp_printf(const char *fmt, ...);
int tfp_sprintf(char *s, const char *fmt, ...);

int tfp_format(void *putp, void (*putf) (void *, char), const char *fmt, va_list va);

#define printf tfp_printf
#define sprintf tfp_sprintf

void printfSupportInit(void);
void setPrintfSerialPort(serialPort_t *serialPort);

#endif	// __PRINTF_H
