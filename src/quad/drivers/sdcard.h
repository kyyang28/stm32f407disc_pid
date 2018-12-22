#ifndef __SDCARD_H
#define __SDCARD_H

#include <stdint.h>
#include <stdbool.h>

typedef struct sdcardConfig_s {
	uint8_t useDma;
}sdcardConfig_t;

typedef struct sdcardMetaData_s {
	uint8_t manufacturerID;
	uint16_t oemID;
	
	char productName[5];
	
	uint8_t productRevisionMajor;
	uint8_t productRevisionMinor;
	uint32_t productSerial;
	
	uint16_t productionYear;
	uint8_t productionMonth;
	
	uint32_t numBlocks;				/* Card capacity in 512-byte blocks */
}sdcardMetaData_t;

typedef enum {
	SDCARD_BLOCK_OPERATION_READ,		// 0
	SDCARD_BLOCK_OPERATION_WRITE,		// 1
	SDCARD_BLOCK_OPERATION_ERASE		// 2
}sdcardBlockOperation_e;

typedef enum {
	SDCARD_OPERATION_IN_PROGRESS,		// 0
	SDCARD_OPERATION_BUSY,				// 1
	SDCARD_OPERATION_SUCCESS,			// 2
	SDCARD_OPERATION_FAILURE			// 3
}sdcardOperationStatus_e;

typedef void (*sdcard_operationCompleteCallback_c)(sdcardBlockOperation_e operation, uint32_t blockIndex, uint8_t *buffer, uint32_t callbackData);

bool sdcard_isInserted(void);
void sdcardInsertionDetectInit(void);
void sdcard_init(bool useDMA);

sdcardOperationStatus_e sdcard_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount);
sdcardOperationStatus_e sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);
bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData);

bool sdcard_poll(void);

#endif	// __SDCARD_H
