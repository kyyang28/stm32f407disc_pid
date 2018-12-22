
#include <stdint.h>
#include "serial.h"

static serialPort_t *rxSerial6TestPort;

void rxSerial6TestInit(void)
{
	serialPortConfig_t *rxSerialTestPortConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
	if (!rxSerialTestPortConfig)
		return;
	
	/* gpsPortConfig->identifier = SERIAL_PORT_USART6 */
	rxSerial6TestPort = openSerialPort(rxSerialTestPortConfig->identifier, FUNCTION_RX_SERIAL, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
	if (!rxSerial6TestPort)
		return;
}

void rxSerial6TestWrite(uint8_t ch)
{
	serialWrite(rxSerial6TestPort, ch);
}

void rxSerial6TestPrint(const char *str)
{
	serialPrint(rxSerial6TestPort, str);
}
