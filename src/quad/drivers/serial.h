#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include "io.h"
#include "resource.h"
#include "target.h"

#define SERIAL_PORT_MAX_INDEX		RESOURCE_SOFT_OFFSET
#define SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(x) (((x) <= SERIAL_PORT_USART8) ? (x) : (RESOURCE_SOFT_OFFSET + ((x) - SERIAL_PORT_SOFTSERIAL1)))

typedef struct serialPinConfig_s {
	ioTag_t ioTagTx[SERIAL_PORT_MAX_INDEX];
	ioTag_t ioTagRx[SERIAL_PORT_MAX_INDEX];
}serialPinConfig_t;

typedef enum {
	SERIAL_PORT_NONE		= -1,
	SERIAL_PORT_USART1		= 0,
	SERIAL_PORT_USART2,
	SERIAL_PORT_USART3,
	SERIAL_PORT_USART4,
	SERIAL_PORT_USART5,
	SERIAL_PORT_USART6,
	SERIAL_PORT_USART7,
	SERIAL_PORT_USART8,
	SERIAL_PORT_USB_VCP		= 20,
	SERIAL_PORT_SOFTSERIAL1	= 30,
	SERIAL_PORT_SOFTSERIAL2
}serialPortIdentifier_e;

typedef enum {
	FUNCTION_NONE						= 0,
	FUNCTION_MSP						= (1 << 0),		// 1
	FUNCTION_GPS						= (1 << 1),		// 2
    FUNCTION_TELEMETRY_FRSKY     		= (1 << 2),  // 4
    FUNCTION_TELEMETRY_HOTT      		= (1 << 3),  // 8
    FUNCTION_TELEMETRY_LTM       		= (1 << 4),  // 16
    FUNCTION_TELEMETRY_SMARTPORT 		= (1 << 5),  // 32
    FUNCTION_RX_SERIAL           		= (1 << 6),  // 64
    FUNCTION_BLACKBOX            		= (1 << 7),  // 128
    FUNCTION_TELEMETRY_MAVLINK   		= (1 << 9),  // 512
    FUNCTION_ESC_SENSOR          		= (1 << 10), // 1024
    FUNCTION_VTX_SMARTAUDIO      		= (1 << 11), // 2048
    FUNCTION_TELEMETRY_IBUS      		= (1 << 12), // 4096
    FUNCTION_VTX_TRAMP           		= (1 << 13), // 8192
}serialPortFunction_e;

typedef enum {
	BAUD_AUTO = 0,
	BAUD_9600,
	BAUD_19200,
	BAUD_38400,
	BAUD_57600,
	BAUD_115200,
	BAUD_230400,
	BAUD_250000,
	BAUD_400000,
	BAUD_460800,
	BAUD_500000,
	BAUD_921600,
	BAUD_1000000,
	BAUD_1500000,
	BAUD_2000000,
	BAUD_2470000,
}baudRate_e;

extern const uint32_t baudRates[];

typedef enum {
	MODE_RX = 1 << 0,
	MODE_TX = 1 << 1,
	MODE_RXTX = MODE_RX | MODE_TX
}portMode_t;

typedef enum {
	SERIAL_NOT_INVERTED = 0 << 0,
	SERIAL_INVERTED		= 1 << 0,
	SERIAL_STOPBITS_1	= 0 << 1,
	SERIAL_STOPBITS_2	= 1 << 1,
	SERIAL_PARITY_NO	= 0 << 2,
	SERIAL_PARITY_EVEN	= 1 << 2,
	SERIAL_UNIDIR		= 0 << 3,
	SERIAL_BIDIR		= 1 << 3,
	
    /*
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
	SERIAL_BIDIR_OD		= 0 << 4,
	SERIAL_BIDIR_PP		= 1 << 4
}portOptions_t;

typedef struct serialPortConfig_s {
	uint16_t functionMask;
	serialPortIdentifier_e identifier;
	uint8_t msp_baudrateIndex;
	uint8_t gps_baudrateIndex;
	uint8_t blackbox_baudrateIndex;
	uint8_t telemetry_baudrateIndex;
}serialPortConfig_t;

typedef struct serialConfig_s {
	serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];
	uint16_t serial_update_rate_hz;
	uint8_t reboot_character;				// default 'R'
}serialConfig_t;

typedef void (*serialReceiveCallbackPtr)(uint16_t data);	// used by serial drivers to return frames to app

typedef struct serialPort_s {
	const struct serialPortVTable *vTable;
	
	uint8_t identifier;
	portMode_t mode;
	portOptions_t options;
	
	uint32_t baudRate;
	
	uint32_t rxBufferSize;
	uint32_t txBufferSize;
	volatile uint8_t *rxBuffer;
	volatile uint8_t *txBuffer;
	uint32_t rxBufferHead;
	uint32_t rxBufferTail;
	uint32_t txBufferHead;
	uint32_t txBufferTail;
	
	serialReceiveCallbackPtr rxCallback;
}serialPort_t;

/* Runtime */
typedef struct serialPortUsage_s {
	serialPortIdentifier_e identifier;
	serialPort_t *serialPort;
	serialPortFunction_e function;
}serialPortUsage_t;

struct serialPortVTable {
	void (*serialWrite)(serialPort_t *instance, uint8_t ch);
	uint32_t (*serialTotalRxWaiting)(const serialPort_t *instance);
	uint32_t (*serialTotalTxFree)(const serialPort_t *instance);
	uint8_t (*serialRead)(serialPort_t *instance);
	void (*serialSetBaudRate)(serialPort_t *instance, uint32_t baudRate);
	bool (*isSerialTransmitBufferEmpty)(const serialPort_t *instance);
	void (*setMode)(serialPort_t *instance, portMode_t mode);
	void (*writeBuf)(serialPort_t *instance, const void *data, int count);
	
	/* original functions used to buffer large writes */
	void (*beginWrite)(serialPort_t *instance);
	void (*endWrite)(serialPort_t *instance);
};

extern const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];

void serialInit(serialConfig_t *initSerialConfig);
serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function);
serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function);
int findSerialPortIndexByIdentifier(serialPortIdentifier_e identifier);

serialPort_t *openSerialPort(
	serialPortIdentifier_e identifier,
	serialPortFunction_e function,
	serialReceiveCallbackPtr rxCallback,
	uint32_t baudRate,
	portMode_t mode,
	portOptions_t options);

void serialWrite(serialPort_t *instance, uint8_t ch);
bool isSerialTransmitBufferEmpty(const serialPort_t *instance);
void serialPrint(serialPort_t *instance, const char *str);

#endif	// __SERIAL_H
