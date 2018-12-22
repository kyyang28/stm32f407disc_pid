#ifndef __PRINTF_H
#define __PRINTF_H

#include "stdio.h"
#include "rxSerial1Test.h"

struct __FILE {
    int dummy;
};

FILE __stdout;

int fputc(int ch, FILE *f);

#endif	// __PRINTF_H
