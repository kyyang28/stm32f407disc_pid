
#include "printf.h"

//FILE __stdout;

int fputc(int ch, FILE *f)
{
    /* Send byte to USART */
//	gpsWrite(ch);
	rxSerial1TestWrite(ch);
//	rxSerial3TestWrite(ch);
//	rxSerial6TestWrite(ch);
    
    /* If everything is OK, you have to return character written */
    return ch;
    /* If character is not correct, you can return EOF (-1) to stop writing */
    //return -1;
}
