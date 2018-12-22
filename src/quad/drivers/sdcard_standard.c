
#include <stdio.h>
#include <string.h>
#include "sdcard_standard.h"

#define MIN(a, b)		((a) < (b) ? (a) : (b))

/**
 * Read a bitfield from an array of bits (the bit at index 0 being the most-significant bit of the first
 * byte in the buffer)
 */
uint32_t readBitfield(uint8_t *buffer, unsigned bitIndex, unsigned bitLen)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	uint32_t result = 0;
	unsigned bitInByteOffset = bitIndex % 8;		// bitInByteOffset = 7
//	printf("bitInByteOffset: %u\r\n", bitInByteOffset);	// bitIndex = 127, bitIndex % 8 = 7
	uint8_t bufferByte;
	
	/* Print contents of buffer */
//	printf("buffer: ");
//	for (int i = 0; i < strlen((const char *)buffer); i++) {
//		printf("0x%x ", buffer[i]);
//	}
//	printf("\r\n");
	
	/* Print buffer[i] addresses, where i = 0 to 16 */
//	for (int i = 0; i < 16; i++) {
//		printf("buffer[%d] before: 0x%x\r\n", i, (unsigned int)&buffer[i]);		// 0x200015e0
//	}
	buffer += bitIndex / 8;		// bitIndex / 8 = 127 / 8 = 15
//	printf("buffer after: 0x%x\r\n", (unsigned int)buffer);			// 0x200015ef
	
//	printf("buffer: ");
//	for (int i = 0; i < 16; i++) {
//		printf("0x%x ", buffer[i]);
//	}
//	printf("\r\n");
		
	/* Align the bitfield to be read to the top of the buffer */
	bufferByte = *buffer << bitInByteOffset;
//	printf("*buffer: 0x%x\r\n", *buffer);
//	printf("*buffer << bitInByteOffset: %u\r\n", *buffer << bitInByteOffset);
//	printf("bufferByte: %u\r\n", bufferByte);
	
	while (bitLen > 0) {
		unsigned bitsThisLoop = MIN(8 - bitInByteOffset, bitLen);	// bitsThisLoop = 1
//		printf("bitsThisLoop: %u\r\n", bitsThisLoop);
		result = (result << bitsThisLoop) | (bufferByte >> (8 - bitsThisLoop));	// result = 1
//		printf("result: %u\r\n", result);
		
		buffer++;
//		printf("buffer address: 0x%x\r\n", (unsigned int)&buffer);
//		printf("*buffer: 0x%x\r\n", *buffer);
		bufferByte = *buffer;
		
		bitLen -= bitsThisLoop;
		bitInByteOffset = 0;
	}
	
	return result;
}
