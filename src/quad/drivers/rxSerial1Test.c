
#include <stdint.h>
#include "serial.h"

static serialPort_t *rxSerial1TestPort;

void rxSerial1TestInit(void)
{
	serialPortConfig_t *rxSerialTestPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_FRSKY);
	if (!rxSerialTestPortConfig)
		return;
	
	/* gpsPortConfig->identifier = SERIAL_PORT_USART1 */
	rxSerial1TestPort = openSerialPort(rxSerialTestPortConfig->identifier, FUNCTION_TELEMETRY_FRSKY, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
	if (!rxSerial1TestPort)
		return;
}

void rxSerial1TestWrite(uint8_t ch)
{
	serialWrite(rxSerial1TestPort, ch);
}

void rxSerial1TestPrint(const char *str)
{
	serialPrint(rxSerial1TestPort, str);
}
