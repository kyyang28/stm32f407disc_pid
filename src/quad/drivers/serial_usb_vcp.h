#ifndef __SERIAL_USB_VCP_H
#define __SERIAL_USB_VCP_H

#include "serial.h"

typedef struct {
	serialPort_t port;
	
	/* Buffer used during bulk writes */
	uint8_t txBuf[20];
	uint8_t txAt;
	
	/* Set if the port is in bulk write mode and can buffer */
	bool buffering;
}vcpPort_t;

serialPort_t *usbVcpOpen(void);

#endif	// __SERIAL_USB_VCP_H
