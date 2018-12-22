
#include "serial.h"
//#include "printf.h"

static serialPort_t *gpsPort;

void gpsInit(void)
{
	serialPortConfig_t *gpsPortConfig = findSerialPortConfig(FUNCTION_GPS);
	if (!gpsPortConfig)
		return;
	
	/* gpsPortConfig->identifier = SERIAL_PORT_USART2 */
	gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
	if (!gpsPort)
		return;
}

//void gpsSetPrintfSerialPort(void)
//{
//	setPrintfSerialPort(gpsPort);
//}

void gpsWrite(uint8_t ch)
{
	serialWrite(gpsPort, ch);
}

void gpsPrint(const char *str)
{
	serialPrint(gpsPort, str);
}
