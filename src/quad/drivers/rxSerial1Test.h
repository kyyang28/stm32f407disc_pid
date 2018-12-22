#ifndef __RXSERIAL1TEST_H
#define __RXSERIAL1TEST_H

#include <stdint.h>

void rxSerial1TestInit(void);
//void gpsSetPrintfSerialPort(void);
void rxSerial1TestWrite(uint8_t ch);
void rxSerial1TestPrint(const char *str);

#endif	// __RXSERIALTEST_H
