
#include <string.h>
#include "serial.h"
#include "msp_serial.h"
#include "printf.h"

static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];

static void resetMspPort(mspPort_t *mspPortToReset, serialPort_t *serialPort)
{
	memset(mspPortToReset, 0, sizeof(mspPort_t));
	mspPortToReset->port = serialPort;
}

void mspSerialAllocatePorts(void)
{
	uint8_t portIndex = 0;
	serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);
	
	while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
		mspPort_t *mspPort = &mspPorts[portIndex];
		if (mspPort->port) {
			portIndex++;
			continue;
		}
		
		serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
		if (serialPort) {
			resetMspPort(mspPort, serialPort);
			portIndex++;
		}
		
		portConfig = findNextSerialPortConfig(FUNCTION_MSP);
	}
}

void mspSerialInit(void)
{
	memset(mspPorts, 0, sizeof(mspPorts));
	mspSerialAllocatePorts();
}

void mspSerialProcess(void)
{
	for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
		mspPort_t * const mspPort = &mspPorts[portIndex];
		if (!mspPort->port) {
			continue;
		}
		
		setPrintfSerialPort(mspPort->port);
	}
}
