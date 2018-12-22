#ifndef __MSP_SERIAL_H
#define __MSP_SERIAL_H

#include "serial.h"

#define MAX_MSP_PORT_COUNT		3
#define MSP_PORT_INBUF_SIZE 	192

typedef enum {
	MSP_IDLE,
	MSP_HEADER_START,
	MSP_HEADER_M,
	MSP_HEADER_ARROW,
	MSP_HEADER_SIZE,
	MSP_HEADER_CMD,
	MSP_COMMAND_RECEIVED
}mspState_e;

typedef struct mspPort_s {
	struct serialPort_s *port;		// null when port unused.
	uint8_t offset;
	uint8_t dataSize;
	uint8_t checksum;
	uint8_t cmdMSP;
	mspState_e c_state;
	uint8_t inBuf[MSP_PORT_INBUF_SIZE];
}mspPort_t;

void mspSerialInit(void);

#endif	// __MSP_SERIAL_H
