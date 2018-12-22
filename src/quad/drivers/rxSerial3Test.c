
/* WARNING: USART3 TX line shares the same I/O pin (PB10) with the SPI2 (SCK) */
#include <stdint.h>
#include "serial.h"

static serialPort_t *rxSerial3TestPort;

void rxSerial3TestInit(void)
{
	serialPortConfig_t *rxSerialTestPortConfig = findSerialPortConfig(FUNCTION_BLACKBOX);
	if (!rxSerialTestPortConfig)
		return;
	
	/* gpsPortConfig->identifier = SERIAL_PORT_USART3 */
	rxSerial3TestPort = openSerialPort(rxSerialTestPortConfig->identifier, FUNCTION_BLACKBOX, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
	if (!rxSerial3TestPort)
		return;
}

void rxSerial3TestWrite(uint8_t ch)
{
	serialWrite(rxSerial3TestPort, ch);
}

void rxSerial3TestPrint(const char *str)
{
	serialPrint(rxSerial3TestPort, str);
}
