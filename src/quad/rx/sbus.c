
#include <stdio.h>			// for debugging purposes
#include "rx.h"
#include "serial.h"
#include "system.h"

/*
 * From oscilloscope
 *
 * FrSKY XSR, X8R SBUS
 * time between each frame: 6ms
 * time to send one frame: 3ms
 *
 * Total will be 6ms + 3ms = 9ms for each frame step
 */

//#define DEBUG_SBUS_RAW_DATA

#define SBUS_TIME_NEEDED_PER_FRAME			3000		// in microseconds

#define SBUS_MAX_CHANNEL					18
#define SBUS_FRAME_SIZE						25

#define SBUS_FRAME_BEGIN_BYTE				0x0F

#define SBUS_BAUDRATE						100000

#define SBUS_PORT_OPTIONS					(SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)

#define SBUS_DIGITAL_CHANNEL_MIN			173			// -100% = 173, which is equivalent to 1000 in PWM signal
#define SBUS_DIGITAL_CHANNEL_MID			992			// 0% = 992, which is equivalent to 1500 in PWM signal
#define SBUS_DIGITAL_CHANNEL_MAX			1812		// 100% = 1812, which is equivalent to 2000 in PWM signal

#define SBUS_FLAG_CHANNEL_17				(1 << 0)
#define SBUS_FLAG_CHANNEL_18				(1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS				(1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE			(1 << 3)

struct sbusFrame_s {
	uint8_t syncByte;				// start byte 0x0F
	
	/* 176 bits of data (11 bits per channel * 16 channels) = 22 bytes (176 bits) */
	unsigned int chan0 : 11;
	unsigned int chan1 : 11;
	unsigned int chan2 : 11;
	unsigned int chan3 : 11;
	unsigned int chan4 : 11;
	unsigned int chan5 : 11;
	unsigned int chan6 : 11;
	unsigned int chan7 : 11;
	unsigned int chan8 : 11;
	unsigned int chan9 : 11;
	unsigned int chan10 : 11;
	unsigned int chan11 : 11;
	unsigned int chan12 : 11;
	unsigned int chan13 : 11;
	unsigned int chan14 : 11;
	unsigned int chan15 : 11;
	uint8_t flags;
	uint8_t endByte;		/* The endByte is 0x00 on FrSKY */
}__attribute__ ((__packed__));

/* IMPORTANT: members in union share the same memory space
 * 
 * once FC receives the sbus data and assign to bytes[SBUS_FRAME_SIZE] array,
 * frame structure can access and parse the SBUS data
 */
typedef union {
	uint8_t bytes[SBUS_FRAME_SIZE];
	struct sbusFrame_s frame;
}sbusFrame_t;

static sbusFrame_t sbusFrame;

static bool sbusFrameDone = false;

static uint32_t sbusChannelData[SBUS_MAX_CHANNEL];

static uint16_t sbusReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
	UNUSED(rxRuntimeConfig);
	
    // Linear fitting values read from OpenTX-ppmus and comparing with values received by X4R
    // http://www.wolframalpha.com/input/?i=linear+fit+%7B173%2C+988%7D%2C+%7B1812%2C+2012%7D%2C+%7B993%2C+1500%7D
	
	/* Convert sbus raw channel data to the recognisable PWM data between 988 and 2012 with mid value 1500
	 * 
	 * y = ax + b
	 * 
	 * since x_1 = 173, y_1 = 988; x_2 = 993, y_2 = 1500; x_3 = 1812, y_3 = 2012
	 *
	 * a * x_1 + b = y_1
	 * a * x_3 + b = y_3
	 * ==> 173a + b = 988
	 *     1812a + b = 2012
	 * ==> a = 0.62477
	 *     b = 879.91458
	 * ==> y = 0.62477x + 879.91458
	 * In this case, x represents the sbus raw channel data (sbusChannelData[chan]), y represents the mapped PWM data between 998 and 2012
	 * We use the approximate equation
	 * 			y = 0.625x + 880
	 *
	 * Therefore, PWM raw data (998 - 2012) = (0.625 * sbusChannelData[chan]) + 880
	 */
	return (5 * sbusChannelData[chan] / 8) + 880;		// (0.625 * sbusChannelData[chan]) + 880;		5 / 8 = 0.625
//	return (0.625 * sbusChannelData[chan]) + 880;		// 0.625 = 625 / 1000 = 5 / 8
}

static uint8_t sbusFrameStatus(void)
{
	if (!sbusFrameDone) {
		return RX_FRAME_PENDING;
	}
	
	sbusFrameDone = false;
	
//	printf("sbusFrame.frame.chan0: %u\r\n", sbusFrame.frame.chan0); // 988, 989, 990 for mid stick value, 193 for low stick value, 1788 for high stick value
//	printf("sbusFrame.frame.chan1: %u\r\n", sbusFrame.frame.chan1);
//	printf("sbusFrame.frame.chan2: %u\r\n", sbusFrame.frame.chan2);
//	printf("sbusFrame.frame.chan3: %u\r\n", sbusFrame.frame.chan3);
//	printf("sbusFrame.frame.chan4: %u\r\n", sbusFrame.frame.chan4);
//	printf("sbusFrame.frame.chan5: %u\r\n", sbusFrame.frame.chan5);
//	printf("sbusFrame.frame.chan6: %u\r\n", sbusFrame.frame.chan6);
//	printf("sbusFrame.frame.chan7: %u\r\n", sbusFrame.frame.chan7);
//	printf("sbusFrame.frame.chan8: %u\r\n", sbusFrame.frame.chan8);
//	printf("sbusFrame.frame.chan9: %u\r\n", sbusFrame.frame.chan9);
//	printf("sbusFrame.frame.chan10: %u\r\n", sbusFrame.frame.chan10);
//	printf("sbusFrame.frame.chan11: %u\r\n", sbusFrame.frame.chan11);
//	printf("sbusFrame.frame.chan12: %u\r\n", sbusFrame.frame.chan12);
//	printf("sbusFrame.frame.chan13: %u\r\n", sbusFrame.frame.chan13);
//	printf("sbusFrame.frame.chan14: %u\r\n", sbusFrame.frame.chan14);
//	printf("sbusFrame.frame.chan15: %u\r\n", sbusFrame.frame.chan15);
//	printf("sbusFrame.frame.flags: %u\r\n\r\n", sbusFrame.frame.flags);
	sbusChannelData[0] = sbusFrame.frame.chan0;
	sbusChannelData[1] = sbusFrame.frame.chan1;
	sbusChannelData[2] = sbusFrame.frame.chan2;
	sbusChannelData[3] = sbusFrame.frame.chan3;
	sbusChannelData[4] = sbusFrame.frame.chan4;
	sbusChannelData[5] = sbusFrame.frame.chan5;
	sbusChannelData[6] = sbusFrame.frame.chan6;
	sbusChannelData[7] = sbusFrame.frame.chan7;
	sbusChannelData[8] = sbusFrame.frame.chan8;
	sbusChannelData[9] = sbusFrame.frame.chan9;
	sbusChannelData[10] = sbusFrame.frame.chan10;
	sbusChannelData[11] = sbusFrame.frame.chan11;
	sbusChannelData[12] = sbusFrame.frame.chan12;
	sbusChannelData[13] = sbusFrame.frame.chan13;
	sbusChannelData[14] = sbusFrame.frame.chan14;
	sbusChannelData[15] = sbusFrame.frame.chan15;

#ifdef DEBUG_SBUS_RAW_DATA	
	printf("sbusChannelData[0]: %u\r\n", sbusChannelData[0]); // 988, 989, 990 for mid stick value, 193 for low stick value, 1788 for high stick value. ROLL
	printf("sbusChannelData[1]: %u\r\n", sbusChannelData[1]); // 985. PITCH
	printf("sbusChannelData[2]: %u\r\n", sbusChannelData[2]); // 195. THROTTLE
	printf("sbusChannelData[3]: %u\r\n", sbusChannelData[3]); // 990. YAW
	printf("sbusChannelData[4]: %u\r\n", sbusChannelData[4]); // 1811. BOXARM
	printf("sbusChannelData[5]: %u\r\n", sbusChannelData[5]); // 992. BEEPER
	printf("sbusChannelData[6]: %u\r\n", sbusChannelData[6]); // 1811
	printf("sbusChannelData[7]: %u\r\n", sbusChannelData[7]); // 172
	printf("sbusChannelData[8]: %u\r\n", sbusChannelData[8]); // 992
	printf("sbusChannelData[9]: %u\r\n", sbusChannelData[9]); // 992
	printf("sbusChannelData[10]: %u\r\n", sbusChannelData[10]); // 992
	printf("sbusChannelData[11]: %u\r\n", sbusChannelData[11]); // 992
	printf("sbusChannelData[12]: %u\r\n", sbusChannelData[12]); // 992
	printf("sbusChannelData[13]: %u\r\n", sbusChannelData[13]); // 992
	printf("sbusChannelData[14]: %u\r\n", sbusChannelData[14]); // 992
	printf("sbusChannelData[15]: %u\r\n", sbusChannelData[15]); // 992
#endif

	if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_17) {
		sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MAX;		// SBUS_DIGITAL_CHANNEL_MAX = 1812
	}else {
		sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MIN;		// SBUS_DIGITAL_CHANNEL_MIN = 173
	}
#ifdef DEBUG_SBUS_RAW_DATA
	printf("sbusChannelData[16]: %u\r\n", sbusChannelData[16]);	// 173
#endif
	
	if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_18) {
		sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MAX;
	}else {
		sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MIN;
	}
#ifdef DEBUG_SBUS_RAW_DATA
	printf("sbusChannelData[17]: %u\r\n\r\n", sbusChannelData[17]);	// 173
#endif
	
	if (sbusFrame.frame.flags & SBUS_FLAG_SIGNAL_LOSS) {
		/* TODO: save sbusStateFlags to debug[0] */
#ifdef DEBUG_SBUS_PACKETS
        sbusStateFlags |= SBUS_STATE_SIGNALLOSS;
        debug[0] = sbusStateFlags;
#endif
	}
	
	if (sbusFrame.frame.flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
		/* internal failsafe is enabled and rx failsafe flag is set */
#ifdef DEBUG_SBUS_PACKETS
        sbusStateFlags |= SBUS_STATE_FAILSAFE;
        debug[0] = sbusStateFlags;
#endif
		/* RX *should* still be sending valid channel data, so use it */
		return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
	}
	
#ifdef DEBUG_SBUS_PACKETS
    debug[0] = sbusStateFlags;
#endif

	return RX_FRAME_COMPLETE;
}

//static void printSBUSDataInBin(uint16_t c)
//{
//	if ((c & 0x80) == 128) {
//		printf("%c", '1');
//	}else {
//		printf("%c", '0');
//	}
//	
//	if ((c & 0x40) == 64) {
//		printf("%c", '1');
//	}else {
//		printf("%c", '0');
//	}
//	
//	if ((c & 0x20) == 32) {
//		printf("%c", '1');
//	}else {
//		printf("%c", '0');
//	}
//	
//	if ((c & 0x10) == 16) {
//		printf("%c", '1');
//	}else {
//		printf("%c", '0');
//	}
//	
//	if ((c & 0x8) == 8) {
//		printf("%c", '1');
//	}else {
//		printf("%c", '0');
//	}
//	
//	if ((c & 0x4) == 4) {
//		printf("%c", '1');
//	}else {
//		printf("%c", '0');
//	}
//	
//	if ((c & 0x2) == 2) {
//		printf("%c", '1');
//	}else {
//		printf("%c", '0');
//	}
//	
//	if ((c & 0x1) == 1) {
//		printf("%c ", '1');
//	}else {
//		printf("%c ", '0');
//	}	
//}

/* SBUS receive ISR callback function */
static void sbusDataReceiveCallBack(uint16_t c)
{
#if 0
	printf("%u ", (uint8_t)c);
#endif
#if 0
	printSBUSDataInBin(c);
#endif
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	static uint8_t sbusFramePosition = 0;
	static uint32_t sbusFrameStartAt = 0;
	uint32_t now = micros();

//	printf("sbusFramePosition: %u, %s, %d\r\n", sbusFramePosition, __FUNCTION__, __LINE__);
//	printf("now: %u, %s, %d\r\n", now, __FUNCTION__, __LINE__);
	
	int32_t sbusFrameTime = now - sbusFrameStartAt;
//	printf("sbusFrameTime: %u, %s, %d\r\n", sbusFrameTime, __FUNCTION__, __LINE__);
	
	/* SBUS_TIME_NEEDED_PER_FRAME = 3000 microseconds
	 *
	 * if sbusFrameTime is longer that 3500 microseconds (3.5 milliseconds), 
	 * then sbusFramePosition sets to the beginning (0)
	 */
	if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
		sbusFramePosition = 0;
	}
	
//	printf("sbusFramePosition: %u, %s, %d\r\n", sbusFramePosition, __FUNCTION__, __LINE__);
	
	if (sbusFramePosition == 0) {
		if (c != SBUS_FRAME_BEGIN_BYTE) {
//			printf("c is not SBUS_FRAME_BEGIN_BYTE\r\n");
			return;
		}
		sbusFrameStartAt = now;		// log the current timestamp (in microseconds) to sbusFrameStartAt when c is equal to SBUS_FRAME_BEGIN_BYTE (0x0F)
//		printf("\r\n\r\n");			// for testing purpose
	}
	
//	printf("sbusFrameStartAt: %u, %s, %d\r\n", sbusFrameStartAt, __FUNCTION__, __LINE__);
	
	/* SBUS_FRAME_SIZE = 25 */
	if (sbusFramePosition < SBUS_FRAME_SIZE) {
		sbusFrame.bytes[sbusFramePosition++] = (uint8_t)c;
		
//		printf("sbusFrame.bytes[%d]: %u\r\n", sbusFramePosition, sbusFrame.bytes[sbusFramePosition]);
#if 0
		printSBUSDataInBin(c);
#endif	
		if (sbusFramePosition < SBUS_FRAME_SIZE) {
			sbusFrameDone = false;
		}else {
			sbusFrameDone = true;
#ifdef DEBUG_SBUS_PACKETS
			debug[2] = sbusFrameTime;
#endif
//			printf("\r\n");
		}
	}
}

bool sbusInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
	for (int i = 0; i < SBUS_MAX_CHANNEL; i++) {
		sbusChannelData[i] = (16 * rxConfig->midrc) / 10 - 1408;		// (16 * rxConfig->midrc) / 10 - 1408 = 992 with midrc = 1500
	}
	
	rxRuntimeConfig->channelCount = SBUS_MAX_CHANNEL;		// SBUS_MAX_CHANNEL = 18
	rxRuntimeConfig->rxRefreshRate = 11000;
	
	rxRuntimeConfig->rcReadRawFn = sbusReadRawRC;
	rxRuntimeConfig->rcFrameStatusFn = sbusFrameStatus;
	
	const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
	if (!portConfig)
		return false;
	
#ifdef TELEMETRY
	bool portShared = telemetryCheckRxPortShared(portConfig);
#else
	bool portShared = false;
#endif
	
//	printf("portConfig->identifier: %d, %s, %d\r\n", portConfig->identifier, __FUNCTION__, __LINE__);		// 0
//	printf("SBUS_BAUDRATE: %d, %s, %d\r\n", SBUS_BAUDRATE, __FUNCTION__, __LINE__);							// 100000
//	printf("rxConfig->sbus_inversion: %d, %s, %d\r\n", rxConfig->sbus_inversion, __FUNCTION__, __LINE__);	// 1
//	printf("rxConfig->halfDuplex: %d, %s, %d\r\n", rxConfig->halfDuplex, __FUNCTION__, __LINE__);			// 0
	/* Open SBUS serial port, using UART1 in this case */
	serialPort_t *sBusPort = openSerialPort(portConfig->identifier,
									FUNCTION_RX_SERIAL,
									sbusDataReceiveCallBack,
									SBUS_BAUDRATE,
									portShared ? MODE_RXTX : MODE_RX,
									SBUS_PORT_OPTIONS | (rxConfig->sbus_inversion ? SERIAL_INVERTED : 0) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
									);
	
#ifdef TELEMETRY
	if (portShared) {
		telemetrySharedPort = sBusPort;
	}
#endif
	
//	printf("The size of sbusFrame: %u, %s, %d\r\n", sizeof(sbusFrame), __FUNCTION__, __LINE__);		// sizeof(sbusFrame) = 25
//	printf("The address of sBusPort: 0x%x, %s, %d\r\n", (uint32_t)sBusPort, __FUNCTION__, __LINE__);	// 0x20000074
	return sBusPort != NULL;
}
