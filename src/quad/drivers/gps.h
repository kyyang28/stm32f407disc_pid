#ifndef __GPS_H
#define __GPS_H

void gpsInit(void);
//void gpsSetPrintfSerialPort(void);
void gpsWrite(uint8_t ch);
void gpsPrint(const char *str);

#endif	// __GPS_H
