
#include <stdio.h>			// debugging purposes
#include "platform.h"

#ifdef USE_SDCARD

#include "io.h"
#include "bus_spi.h"
#include "system.h"			// delay(), delayMicroseconds(), millis()

#include "sdcard.h"
#include "sdcard_standard.h"

#include "dma.h"

typedef enum {
	/* In these states, SD card runs at the initialisation 400kHz clockspeed */
	SDCARD_STATE_NOT_PRESENT = 0,
	SDCARD_STATE_RESET,											// 1
	SDCARD_STATE_CARD_INIT_IN_PROGRESS,							// 2
	SDCARD_STATE_INITIALISATION_RECEIVE_CID,					// 3
	
	/* In these states, SD card runs at full clock speed */
	SDCARD_STATE_READY,											// 4
	SDCARD_STATE_READING,										// 5
	SDCARD_STATE_SENDING_WRITE,									// 6
	SDCARD_STATE_WAITING_FOR_WRITE,								// 7
	SDCARD_STATE_WRITING_MULTIPLE_BLOCKS,						// 8
	SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE					// 9
}sdcardState_e;

typedef enum {
	SDCARD_RECEIVE_SUCCESS,
	SDCARD_RECEIVE_BLOCK_IN_PROGRESS,
	SDCARD_RECEIVE_ERROR
}sdcardReceiveBlockStatus_e;

typedef struct sdcard_t {
	struct {
		uint8_t *buffer;
		uint32_t blockIndex;
		uint8_t chunkIndex;
		
		sdcard_operationCompleteCallback_c callback;
		uint32_t callbackData;
	}pendingOperation;
	
	uint32_t operationStartTime;
	
	uint8_t failureCount;
	
	uint8_t version;
	
	bool highCapacity;
	
	uint32_t multiWriteNextBlock;
	uint32_t multiWriteBlocksRemain;
	
	sdcardState_e state;
	
	sdcardMetaData_t metaData;
	
	sdcardCSD_t csd;
}sdcard_t;

static sdcard_t sdcard;

#ifdef SDCARD_DMA_CHANNEL_TX
	static bool useDMAForTx;
#else
	/* DMA channel is not available so we can hard-code this to allow the non-DMA paths to be stripped by optimisation */
	static const bool useDMAForTx = false;
#endif

#ifdef SDCARD_DETECT_PIN
static IO_t sdcardDetectPin = IO_NONE;
#endif

static IO_t sdcardCsPin = IO_NONE;

#define SET_CS_HIGH									IOHi(sdcardCsPin)
#define SET_CS_LOW									IOLo(sdcardCsPin)

#define SDCARD_INIT_NUM_DUMMY_BYTES					10
#define SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY		8

/* Chosen so that CMD8 will have the same CRC as CMD0 */
#define SDCARD_IF_COND_CHECK_PATTERN				0xAB

#define SDCARD_MAX_CONSECUTIVE_FAILURES 			8

#define SDCARD_NON_DMA_CHUNK_SIZE					256

#define SDCARD_TIMEOUT_INIT_MILLIS					200

/**
 * Detect if a SD card is physically present in the memory slot.
 * 
 * @return
 *		result = true, card is present (IORead(sdcardDetectPin) returns 0), sdcardDetectPin = PC14
 *		result = false, card is not present (IORead(sdcardDetectPin) returns 1), sdcardDetectPin = PC14
 */
bool sdcard_isInserted(void)
{
	bool result = true;
	
#ifdef SDCARD_DETECT_PIN
	/* According to <<Micro-SD-Storage-Board-Schematic.pdf>>,
	 * when card is not present, card detect pin is connected to 3V3, 
	 * so IORead(sdcardDetectPin) is HIGH level (sdcardDetectPin is CARD DETECT PIN (PC14))
	 */
//	printf("IORead(sdcardDetectPin): %d, %s, %d\r\n", IORead(sdcardDetectPin), __FUNCTION__, __LINE__);
	result = IORead(sdcardDetectPin) != 0;
//	printf("result: %d, %s, %d\r\n", result, __FUNCTION__, __LINE__);
	
#ifdef SDCARD_DETECT_INVERTED		// for WAVESHARE SDCARD module, CD(card detect) pin is inverted
	result = !result;
#endif
#endif
	
	return result;
}

void sdcardInsertionDetectInit(void)
{
#ifdef SDCARD_DETECT_PIN			// SDCARD_DETECT_PIN = PC14 in this case
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	sdcardDetectPin = IOGetByTag(IO_TAG(SDCARD_DETECT_PIN));
	IOInit(sdcardDetectPin, OWNER_SDCARD_DETECT, 0);
	IOConfigGPIO(sdcardDetectPin, IOCFG_IPU);
#endif
}

/**
 * SD card initialisation process. This must be called first before any other sdcard_ routine
 */
void sdcard_init(bool useDMA)
{
//	printf("useDMA: %d, %s, %d\r\n", useDMA, __FUNCTION__, __LINE__);
#ifdef SDCARD_DMA_CHANNEL_TX
	useDMAForTx = useDMA;
	if (useDMAForTx) {
		dmaInit(dmaGetIdentifier(SDCARD_DMA_CHANNEL_TX), OWNER_SDCARD, 0);
	}
#else
	/* DMA is not available */
	(void) useDMA;
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
#endif
	
#ifdef SDCARD_SPI_CS_PIN
	sdcardCsPin = IOGetByTag(IO_TAG(SDCARD_SPI_CS_PIN));
	IOInit(sdcardCsPin, OWNER_SDCARD_CS, 0);
	IOConfigGPIO(sdcardCsPin, SPI_IO_CS_CFG);
#endif	// SDCARD_SPI_CS_PIN
	
	/* Maximum frequency is initialised to roughly 400Khz */
	spiSetDivisor(SDCARD_SPI_INSTANCE, SDCARD_SPI_INITIALISATION_CLOCK_DIVISOR);
	
	/* SD card requires 1ms maximum initialisation delay after power is applied to it */
	delay(1000);
	
	/* delay(1000) postpones at least 74 dummy clock cycles with CS high so the SD card can start up */
	SET_CS_HIGH;
	
	spiTransfer(SDCARD_SPI_INSTANCE, NULL, NULL, SDCARD_INIT_NUM_DUMMY_BYTES);	// SDCARD_INIT_NUM_DUMMY_BYTES = 10
	
	/* Wait for the transmission to finish before we enable the SD card, so it receives the required number of cycles */
	int time = 100000;
	while (spiIsBusBusy(SDCARD_SPI_INSTANCE)) {
		if (time-- == 0) {
			sdcard.state = SDCARD_STATE_NOT_PRESENT;
			sdcard.failureCount++;
			return;
		}
	}
	
	sdcard.operationStartTime = millis();
	sdcard.state = SDCARD_STATE_RESET;			// SDCARD_STATE_RESET = 1
	sdcard.failureCount = 0;
//	printf("sdcard.state: %d, %s, %d\r\n", sdcard.state, __FUNCTION__, __LINE__);
}

static void sdcard_select(void)
{
	SET_CS_LOW;
}

static void sdcard_deselect(void)
{
	/*
	 * As per the SD-card spec, give the card 8 dummy clocks so it can finish its operation
	 * spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF)
	 */
	while (spiIsBusBusy(SDCARD_SPI_INSTANCE)) {
	}
	
	SET_CS_HIGH;
}

/*
 * Returns true if the card is ready to accept read/write commands.
 */
static bool sdcard_isReady(void)
{
	return sdcard.state == SDCARD_STATE_READY || sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
}

/**
 * The SD card spec requires 8 clock cycles to be sent by us on the bus after most commands so it can finish its
 * processing of that command. The easiest way for us to do this is to just wait for the bus to become idle before
 * we transmit a command, sending at least 8-bits onto the bus when we do so.
 */
static bool sdcard_waitForIdle(int maxBytesToWait)
{
	while (maxBytesToWait > 0) {
		uint8_t b = spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
		if (b == 0xFF) {
			return true;
		}
		maxBytesToWait--;
	}
	
	return false;
}

/**
 * Wait for up to maxDelay 0xFF idle bytes to arrive from the card, returning the first non-idle byte found.
 *
 * Returns 0xFF on failure.
 */
static uint8_t sdcard_waitForNonIdleByte(int maxDelay)
{
	for (int i = 0; i < maxDelay + 1; i++) {	// + 1 so we can wait for maxDelay '0xFF' bytes before reading a response byte afterwards
		uint8_t response = spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
		if (response != 0xFF) {
//			printf("response: %u\r\n", response);
			return response;
		}
	}
	
	/* returns 0xFF on failure */
	return 0xFF;
}

static uint8_t sdcard_sendCommand(uint8_t commandCode, uint32_t commandArgument)
{
	uint8_t command[6] = {
		0x40 | commandCode,
		commandArgument >> 24,
		commandArgument >> 16,
		commandArgument >> 8,
		commandArgument,
		0x95 /* static CRC. This CRC is valid for CMD0 with a 0 argument, and CMD8 with 0x1AB argument, which are the only commands that require a CRC */
	};
	
	/* Go ahead and send the command even if the card isn't idle if this is the reset command */
	if (!sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY) && commandCode != SDCARD_COMMAND_GO_IDLE_STATE) {
		return 0xFF;
	}
	
	/* Send command on the CMD line */
	spiTransfer(SDCARD_SPI_INSTANCE, NULL, command, sizeof(command));
	
    /*
     * The card can take up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes to send the response, in the meantime
     * it'll transmit 0xFF filler bytes.
     */
	return sdcard_waitForNonIdleByte(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY);
}

static uint8_t sdcard_sendAppCommand(uint8_t commandCode, uint32_t commandArgument)
{
	sdcard_sendCommand(SDCARD_COMMAND_APP_CMD, 0);
	
	return sdcard_sendCommand(commandCode, commandArgument);
}

/**
 * Sends an IF_COND message to the card to check its version and validate its voltage requirements. Sets the global
 * sdCardVersion with the detected version (0, 1, or 2) and returns true if the card is compatible.
 */
static bool sdcard_validateInterfaceCondition(void)
{
	uint8_t ifCondReply[4];
	sdcard.version = 0;
	
	sdcard_select();
	
	/* (SDCARD_VOLTAGE_ACCEPTED_2_7_to_3_6 << 8) | SDCARD_IF_COND_CHECK_PATTERN = 0x1 << 8 | 0xAB = 0x100 | 0xAB = 0x1AB */
	uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SEND_IF_COND, (SDCARD_VOLTAGE_ACCEPTED_2_7_to_3_6 << 8) | SDCARD_IF_COND_CHECK_PATTERN);
	
//	printf("status: 0x%x\r\n", status);		// status = 0x01 (SDCARD_R1_STATUS_BIT_IDLE)
	
	/* Do not deselect the card right away, because we'll want to read the rest of its reply if it's a V2 card */
	
	/* if status = 0x05 (SDCARD_R1_STATUS_BIT_ILLEGAL_COMMAND | SDCARD_R1_STATUS_BIT_IDLE = 0x04 | 0x01 = 0x05) */
	if (status == (SDCARD_R1_STATUS_BIT_ILLEGAL_COMMAND | SDCARD_R1_STATUS_BIT_IDLE)) {
		/* V1 card do not support this command */
		sdcard.version = 1;
	}else if (status == SDCARD_R1_STATUS_BIT_IDLE) {
		spiTransfer(SDCARD_SPI_INSTANCE, ifCondReply, NULL, sizeof(ifCondReply));

		/* CMD8 reponse is 0x000001AB (R1 + trailing 32-bit data) */
//		printf("ifCondReply[0]: 0x%x\r\n", ifCondReply[0]);		// ifCondReply[0] = 0x00
//		printf("ifCondReply[1]: 0x%x\r\n", ifCondReply[1]);		// ifCondReply[1] = 0x00
//		printf("ifCondReply[2]: 0x%x\r\n", ifCondReply[2]);		// ifCondReply[2] = 0x01
//		printf("ifCondReply[3]: 0x%x\r\n", ifCondReply[3]);		// ifCondReply[3] = 0xAB
		
        /*
         * We don't bother to validate the SDCard's operating voltage range since the spec requires it to accept our
         * 3.3V, but do check that it echoed back our check pattern properly.
         */
		if (ifCondReply[3] == SDCARD_IF_COND_CHECK_PATTERN) {	// SDCARD_IF_COND_CHECK_PATTERN = 0xAB
			sdcard.version = 2;
		}
	}
	
	sdcard_deselect();
	
	return sdcard.version > 0;
}

/**
 * Check if the SD Card has completed its startup sequence. Must be called with sdcard.state == SDCARD_STATE_INITIALIZATION.
 *
 * Returns true if the card has finished its init process.
 */
static bool sdcard_checkInitDone(void)
{
	sdcard_select();

	/* sdcard.version == 2 is true, 1 << 30 is 0x40000000 */
	uint8_t status = sdcard_sendAppCommand(SDCARD_ACOMMAND_SEND_OP_COND, sdcard.version == 2 ? 1 << 30 /* We support high capacity cards */ : 0);
	
	sdcard_deselect();
	
	return status == 0x00;
}

static bool sdcard_readOCRRegister(uint32_t *result)
{
	sdcard_select();
	
	/* Send OCR command */
	uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_READ_OCR, 0);
	
	uint8_t response[4];
	
	/* Receive the contents of OCR register and store to response[4] array */
	spiTransfer(SDCARD_SPI_INSTANCE, response, NULL, sizeof(response));
	
	if (status == 0) {
		sdcard_deselect();
		
		/* 
		 * response[0] = 0xC0
		 * response[1] = 0xFF
		 * response[2] = 0x80
		 * response[3] = 0x00
		 */
//		printf("res[0], res[1], res[2], res[3]: %u, %u, %u, %u\r\n", response[0], response[1], response[2], response[3]);
		*result = (response[0] << 24) | (response[1] << 16) | (response[2] << 8) | response[3];
		
		return true;
	}else {
		sdcard_deselect();
		
		return false;
	}
}

/**
 * Handle a failure of a SD card operation by resetting the card back to its initialization phase.
 *
 * Increments the failure counter, and when the failure threshold is reached, disables the card until
 * the next call to sdcard_init().
 */
static void sdcard_reset(void)
{
	if (!sdcard_isInserted()) {
		sdcard.state = SDCARD_STATE_NOT_PRESENT;
		return;
	}
	
	if (sdcard.state >= SDCARD_STATE_READY) {
		spiSetDivisor(SDCARD_SPI_INSTANCE, SDCARD_SPI_INITIALISATION_CLOCK_DIVISOR);
	}
	
	sdcard.failureCount++;
	
	if (sdcard.failureCount >= SDCARD_MAX_CONSECUTIVE_FAILURES) {
		sdcard.state = SDCARD_STATE_NOT_PRESENT;
	}else {
		sdcard.operationStartTime = millis();
		sdcard.state = SDCARD_STATE_RESET;
	}
}

/**
 * Attempt to receive a data block from the SD card.
 *
 * Return true on success, otherwise the card has not responded yet and you should retry later.
 */
static sdcardReceiveBlockStatus_e sdcard_receiveDataBlock(uint8_t *buffer, int count)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	uint8_t dataToken = sdcard_waitForNonIdleByte(8);
//	printf("dataToken: %u\r\n", dataToken);
	if (dataToken == 0xFF) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return SDCARD_RECEIVE_BLOCK_IN_PROGRESS;
	}
	
	if (dataToken != SDCARD_SINGLE_BLOCK_READ_START_TOKEN) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		return SDCARD_RECEIVE_ERROR;
	}
	
	spiTransfer(SDCARD_SPI_INSTANCE, buffer, NULL, count);
	
	/* discard trailing CRC, we don't care */
	spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
	spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
	
	return SDCARD_RECEIVE_SUCCESS;
}

static bool sdcard_fetchCSD(void)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	uint32_t readBlockLen, blockCount, blockCountMult;
	uint64_t capacityBytes;
	
	sdcard_select();
	
    /* The CSD command's data block should always arrive within 8 idle clock cycles (SD card spec). 
	 * This is because the information about card latency is stored in the CSD register itself, 
	 * so we can't use that yet!
     */
	bool success = sdcard_sendCommand(SDCARD_COMMAND_SEND_CSD, 0) == 0
		&& sdcard_receiveDataBlock((uint8_t *)&sdcard.csd, sizeof(sdcard.csd)) == SDCARD_RECEIVE_SUCCESS
		&& SDCARD_GET_CSD_FIELD(sdcard.csd, 1, TRAILER) == 1;
//	printf("success: %d, %s, %d\r\n", success, __FUNCTION__, __LINE__);
//	printf("size: %u\r\n", sizeof(sdcard.csd.data)/sizeof(sdcard.csd.data[0]));
//	printf("data: ");
//	for (int i = 0; i < sizeof(sdcard.csd.data)/sizeof(sdcard.csd.data[0]); i++) {
//		printf("0x%x ", sdcard.csd.data[i]);
//	}
//	printf("\r\n");
		
	if (success) {
		switch (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSD_STRUCTURE_VER)) {
			case SDCARD_CSD_STRUCTURE_VERSION_1:	// for low capacity cards (max 2GB)
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
				/* Block size in bytes (doesn't have to be 512) */
				readBlockLen = 1 << SDCARD_GET_CSD_FIELD(sdcard.csd, 1, READ_BLOCK_LEN);
				blockCountMult = 1 << (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSIZE_MULT) + 2);
				blockCount = (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSIZE) + 1) * blockCountMult;
//				printf("readBlockLen: %u\r\n", readBlockLen);
//				printf("blockCountMult: %u\r\n", blockCountMult);
//				printf("blockCount: %u\r\n", blockCount);
			
				/* We could do this in 32 bits, but it makes the 2GB case awkward */
				capacityBytes = (uint64_t) blockCount * readBlockLen;
				
				/* Re-express that capacity (max 2GB) in our standard 512-byte block size */
				sdcard.metaData.numBlocks = capacityBytes / SDCARD_BLOCK_SIZE;
				break;
			
			case SDCARD_CSD_STRUCTURE_VERSION_2:		// for high capacity cards
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
				/* SDCARD_GET_CSD_FIELD(sdcard.csd, 2, CSIZE) = 30386 */
//				printf("SDCARD_GET_CSD_FIELD(sdcard.csd, 2, CSIZE): %u\r\n", SDCARD_GET_CSD_FIELD(sdcard.csd, 2, CSIZE));
				/* sdcard.metaData.numBlocks = 31116288 */
				sdcard.metaData.numBlocks = (SDCARD_GET_CSD_FIELD(sdcard.csd, 2, CSIZE) + 1) * 1024;
//				printf("sdcard.metaData.numBlocks: %u\r\n", sdcard.metaData.numBlocks);
				break;
			
			default:
				success = false;
		}
	}
	
	sdcard_deselect();
	
	return success;
}

static bool sdcard_receiveCID(void)
{
	uint8_t cid[16];
	
	if (sdcard_receiveDataBlock(cid, sizeof(cid)) != SDCARD_RECEIVE_SUCCESS) {
		return false;
	}
	
//	printf("cid: ");
//	for (int i = 0; i < sizeof(cid)/sizeof(cid[0]); i++) {
//		printf("%u ", cid[i]);
//	}
//	printf("\r\n");

	/* Product_Manual[SanDisk_Secure_Digital_Card].pdf, Table 3-9. CID fields */
	sdcard.metaData.manufacturerID = cid[0];
	sdcard.metaData.oemID = (cid[1] << 8) | cid[2];
	sdcard.metaData.productName[0] = cid[3];
	sdcard.metaData.productName[1] = cid[4];
	sdcard.metaData.productName[2] = cid[5];
	sdcard.metaData.productName[3] = cid[6];
	sdcard.metaData.productName[4] = cid[7];
	sdcard.metaData.productRevisionMajor = cid[8] >> 4;
	sdcard.metaData.productRevisionMinor = cid[8] & 0x0F;
	sdcard.metaData.productSerial = (cid[9] << 24) | (cid[10] << 16) | (cid[11] << 8) | cid[12];
	sdcard.metaData.productionYear = (((cid[13] & 0x0F) << 4) | (cid[14] >> 4)) + 2000;
	sdcard.metaData.productionMonth = cid[14] & 0x0F;

#if 0
	printf("sdcard.metaData.manufacturerID: %u\r\n", sdcard.metaData.manufacturerID);
	printf("sdcard.metaData.oemID: 0x%x\r\n", sdcard.metaData.oemID);
	printf("sdcard.metaData.productName: %s\r\n", sdcard.metaData.productName);
	printf("sdcard.metaData.productRevisionMajor: %u\r\n", sdcard.metaData.productRevisionMajor);
	printf("sdcard.metaData.productRevisionMinor: %u\r\n", sdcard.metaData.productRevisionMinor);
	printf("sdcard.metaData.productSerial: 0x%x\r\n", sdcard.metaData.productSerial);
	printf("sdcard.metaData.productionYear: %u\r\n", sdcard.metaData.productionYear);
	printf("sdcard.metaData.productionMonth: %u\r\n", sdcard.metaData.productionMonth);
#endif
	
	return true;
}

static bool sdcard_setBlockLength(uint32_t blockLen)
{
	sdcard_select();
	
	/* SDCARD_COMMAND_SET_BLOCKLEN = 16, blockLen = 512 normally */
	uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SET_BLOCKLEN, blockLen);
	
	sdcard_deselect();
	
	return status == 0;
}

static bool sdcard_sendDataBlockFinish(void)
{
	/* Send a dummy CRC */
	spiTransferByte(SDCARD_SPI_INSTANCE, 0x00);
	spiTransferByte(SDCARD_SPI_INSTANCE, 0x00);

	uint8_t dataResponseToken = spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
	
    /*
     * Check if the card accepted the write (no CRC error / no address error)
     *
     * The lower 5 bits are structured as follows:
     * | 0 | Status  | 1 |
     * | 0 | x  x  x | 1 |
     *
     * Status:
     * 010 - Data accepted
     * 101 - CRC error
     * 110 - Write error
     */
	return (dataResponseToken & 0x1F) == 0x05;	// check if dataResponseToken stores CRC error, true indicates CRC error
}

/**
 * Send the stop-transmission token to complete a multi-block write.
 *
 * Returns:
 * 		SDCARD_OPERATION_IN_PROGRESS - We're now waiting for that stop to complete, the card will enter
 *									   the SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE state.
 *		SDCARD_OPERATION_SUCCESS	 - The multi-block write finished immediately, the card will enter
 *									   the SDCARD_READY state.
 */
static sdcardOperationStatus_e sdcard_endWriteBlocks(void)
{
	sdcard.multiWriteBlocksRemain = 0;
	
	/* 8 dummy clocks to guarantee N_WR clocks between the last card response and this token */
	spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
	
	spiTransferByte(SDCARD_SPI_INSTANCE, SDCARD_MULTIPLE_BLOCK_WRITE_STOP_TOKEN);
	
	/* Card may choose to raise a busy (non-0xFF) signal after at most N_BR (1 byte) delay */
	if (sdcard_waitForNonIdleByte(1) == 0xFF) {
		sdcard.state = SDCARD_STATE_READY;
		return SDCARD_OPERATION_SUCCESS;
	} else {
		sdcard.state = SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE;
		sdcard.operationStartTime = millis();
		
		return SDCARD_OPERATION_IN_PROGRESS;
	}
}

/**
 * Call periodically for the SD card to perform in-progress transfers.
 *
 * Returns true if the card is ready to accept commands.
 */
bool sdcard_poll(void)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	uint8_t initStatus;
	bool sendComplete;

	/* Referenced by <How to use SDC.pdf, Initialization Procedure for SPI Mode and Data Transfer> */
	doMore:
//	printf("sdcard.state: %d, %s, %d\r\n", sdcard.state, __FUNCTION__, __LINE__);
	switch (sdcard.state) {
		case SDCARD_STATE_RESET:
			sdcard_select();
			
			initStatus = sdcard_sendCommand(SDCARD_COMMAND_GO_IDLE_STATE, 0);
			
			sdcard_deselect();
		
//			printf("initStatus: %u, %s, %d\r\n", initStatus, __FUNCTION__, __LINE__);
			if (initStatus == SDCARD_R1_STATUS_BIT_IDLE) {
				/* Check sdcard voltage and version */
				if (sdcard_validateInterfaceCondition()) {
//					printf("sdcard.version: %u, %s, %d\r\n", sdcard.version, __FUNCTION__, __LINE__);
					sdcard.state = SDCARD_STATE_CARD_INIT_IN_PROGRESS;
//					printf("sdcard.state: %u, %s, %d\r\n", sdcard.state, __FUNCTION__, __LINE__);
					goto doMore;
				}else {
					/* Bad reply/voltage, we ought to refrain from accessing the card */
					sdcard.state = SDCARD_STATE_NOT_PRESENT;			// SDCARD_STATE_NOT_PRESENT = 0
//					printf("sdcard.state: %u, %s, %d\r\n", sdcard.state, __FUNCTION__, __LINE__);
				}
			}
		
			break;
		
		case SDCARD_STATE_CARD_INIT_IN_PROGRESS:
//			printf("SDCARD_STATE_CARD_INIT_IN_PROGRESS!!\r\n");
			/* Send ACMD41 */
			if (sdcard_checkInitDone()) {
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
				if (sdcard.version == 2) {
					/* Check for high capacity card */
					uint32_t ocr;
					
					/* Read OCR register with CMD58 and assign the response to ocr variable */
					if (!sdcard_readOCRRegister(&ocr)) {
						sdcard_reset();
						goto doMore;
					}
						
//					printf("ocr: 0x%x\r\n", ocr);		// ocr = 0xC0FF8000
					sdcard.highCapacity = (ocr & (1 << 30)) != 0;		// sdcard.highCapacity = 1
//					printf("sdcard.highCapacity: %d\r\n", sdcard.highCapacity);
				}else {
					/* Version 1 cards are always low-capacity */
					sdcard.highCapacity = false;
				}
				
				/* Now fetch the CSD and CID registers */
				if (sdcard_fetchCSD()) {
					sdcard_select();
					
					uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SEND_CID, 0);
					if (status == 0) {
						/* Keep the card selected to receive the response block */
						sdcard.state = SDCARD_STATE_INITIALISATION_RECEIVE_CID;
						goto doMore;
					}else {
						sdcard_deselect();
						
						sdcard_reset();
						goto doMore;
					}
				}
			}
			break;
		
		case SDCARD_STATE_INITIALISATION_RECEIVE_CID:
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			if (sdcard_receiveCID()) {
				sdcard_deselect();
				
                /* The spec is a little iffy on what the default block size is for Standard Size cards (it can be changed on
                 * standard size cards) so let's just set it to 512 explicitly so we don't have a problem.
                 */
				if (!sdcard.highCapacity && !sdcard_setBlockLength(SDCARD_BLOCK_SIZE)) {
					sdcard_reset();
					goto doMore;
				}
				
				/* *----------------------------------------------------------------+ */
				/* *-------------- SDCard initialisation is done here --------------+ */
				/* *----------------------------------------------------------------+ */
				
				/* Now we are done with init and we can switch to the full speed clock (<25MHz) */
				spiSetDivisor(SDCARD_SPI_INSTANCE, SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER);
				
				sdcard.multiWriteBlocksRemain = 0;
				
				sdcard.state = SDCARD_STATE_READY;
				
//				printf("sdcard.state: %u\r\n", sdcard.state);
				
				goto doMore;
			}	// else part keeps waiting for the CID to arrive
			break;
		
		case SDCARD_STATE_SENDING_WRITE:
			/* Have we finished sending the write yet? */
			sendComplete = false;
		
#ifdef SDCARD_DMA_CHANNEL_TX
#if defined(USE_HAL_DRIVER)
	/* HAL implementation might be implemented later */
#else
#ifdef SDCARD_DMA_CHANNEL
			if (useDMAForTx && DMA_GetFlagStatus(SDCARD_DMA_CHANNEL_TX, SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG) == SET) {
				DMA_ClearFlag(SDCARD_DMA_CHANNEL_TX, SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG);
#else
            if (useDMAForTx && DMA_GetFlagStatus(SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG) == SET) {
                DMA_ClearFlag(SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG);
#endif
				DMA_Cmd(SDCARD_DMA_CHANNEL_TX, DISABLE);
				
				/* Drain anything left in the Rx FIFO (we didn't read it during the write) */
				while (SPI_I2S_GetFlagStatus(SDCARD_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == SET) {
					SDCARD_SPI_INSTANCE->DR;
				}
				
				/* Wait for the final bit to be transmitted */
				while (spiIsBusBusy(SDCARD_SPI_INSTANCE)) {
				}
				
				SPI_I2S_DMACmd(SDCARD_SPI_INSTANCE, SPI_I2S_DMAReq_Tx, DISABLE);
				
				sendComplete = true;
			}
#endif		
#endif
			/** 
			 * Transmitting without using DMA
			 */
			if (!useDMAForTx) {
				/* Send another chunk 
				 *
				 * SDCARD_NON_DMA_CHUNK_SIZE = 256
				 * SDCARD_BLOCK_SIZE = 512
				 * chunkIndex = SDCARD_BLOCK_SIZE / SDCARD_NON_DMA_CHUNK_SIZE = 512 / 256 = 2
				 */
				spiTransfer(SDCARD_SPI_INSTANCE, NULL, sdcard.pendingOperation.buffer + SDCARD_NON_DMA_CHUNK_SIZE * sdcard.pendingOperation.chunkIndex, SDCARD_NON_DMA_CHUNK_SIZE);
				
				sdcard.pendingOperation.chunkIndex++;
				
				sendComplete = sdcard.pendingOperation.chunkIndex == SDCARD_BLOCK_SIZE / SDCARD_NON_DMA_CHUNK_SIZE;
			}
			
			if (sendComplete) {
				/* Finish up by sending the CRC and checking the SD card's acceptance/rejectance */
				if (sdcard_sendDataBlockFinish()) {
					/* The SD card is now busy committing that write to the card */
					sdcard.state = SDCARD_STATE_WAITING_FOR_WRITE;
					sdcard.operationStartTime = millis();
					
					/* Since we've transmitted the buffer we can go ahead and tell the caller their operation is complete */
					if (sdcard.pendingOperation.callback) {
						sdcard.pendingOperation.callback(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, sdcard.pendingOperation.buffer, sdcard.pendingOperation.callbackData);
					}
				} else {
					/**
					 * Our write was rejected! This could be due to a bad address but we hope not to attempt that, so assume
					 * the card is broken and needs reset.
					 */
					sdcard_reset();
					
					/* Announce write failure */
					if (sdcard.pendingOperation.callback) {
						sdcard.pendingOperation.callback(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, NULL, sdcard.pendingOperation.callbackData);
					}
					
					goto doMore;
				}
			}
			
			break;
		
		case SDCARD_STATE_WAITING_FOR_WRITE:
			if (sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY)) {
#ifdef SDCARD_PROFILING
                profilingComplete = true;
#endif
				/* Assume the card is good if it can complete a write */
				sdcard.failureCount = 0;
				
				/* Still more blocks left to write in a multi-block chain? */
				if (sdcard.multiWriteBlocksRemain > 1) {
					sdcard.multiWriteBlocksRemain--;
					sdcard.multiWriteNextBlock++;
					sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
				} else if (sdcard.multiWriteBlocksRemain == 1) {
					/* This function changes the SD card state for us whether immediately succesful or delayed */
					if (sdcard_endWriteBlocks() == SDCARD_OPERATION_SUCCESS) {
						sdcard_deselect();
					} else {
#ifdef SDCARD_PROFILING
                        // Wait for the multi-block write to be terminated before finishing timing
                        profilingComplete = false;
#endif						
					}
				} else {
					sdcard.state = SDCARD_STATE_READY;
					sdcard_deselect();
				}
				
#ifdef SDCARD_PROFILING
                if (profilingComplete && sdcard.profiler) {
                    sdcard.profiler(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                }
#endif				
			} else if (millis() > sdcard.operationStartTime + SDCARD_TIMEOUT_WRITE_MSEC) {
				/**
				 * The caller has already been told that their write has completed, so they will have discarded
				 * their buffer and have no hope of retrying the operation. But this should be very rare and it allows
				 * them to reuse their buffer milliseconds faster than they otherwise would.
				 */
				sdcard_reset();
				goto doMore;
			}
			break;
		
		case SDCARD_STATE_READING:
//			printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
			switch (sdcard_receiveDataBlock(sdcard.pendingOperation.buffer, SDCARD_BLOCK_SIZE)) {
				case SDCARD_RECEIVE_SUCCESS:
					sdcard_deselect();		// calling sdcard_deselect() from sdcard_readBlock() that return 0 after sending CMD17
					
					sdcard.state = SDCARD_STATE_READY;
					sdcard.failureCount = 0;		// Assume the card is good if it can complete a read

#ifdef SDCARD_PROFILING

#endif				

					/*
				     * sdcard.pendingOperation.callback() function is assigned inside sdcard_readBlock() function
				     */
					if (sdcard.pendingOperation.callback) {
						sdcard.pendingOperation.callback(
							SDCARD_BLOCK_OPERATION_READ,
							sdcard.pendingOperation.blockIndex,
							sdcard.pendingOperation.buffer,
							sdcard.pendingOperation.callbackData
						);
//						printf("%s, %d\r\n", __FUNCTION__, __LINE__);
					}
				
					break;
				
				case SDCARD_RECEIVE_BLOCK_IN_PROGRESS:
					if (millis() <= sdcard.operationStartTime + SDCARD_TIMEOUT_READ_MSEC) {
//						printf("%s, %d\r\n", __FUNCTION__, __LINE__);
						break;		// Timeout not reached yet, so keep waiting
					}
					
					/* Timeout has expired, so fall through to convert to a fatal error */
				
				case SDCARD_RECEIVE_ERROR:
					sdcard_deselect();
				
					sdcard_reset();
				
					if (sdcard.pendingOperation.callback) {
						sdcard.pendingOperation.callback(
							SDCARD_BLOCK_OPERATION_READ,
							sdcard.pendingOperation.blockIndex,
							NULL,
							sdcard.pendingOperation.callbackData
						);
//						printf("%s, %d\r\n", __FUNCTION__, __LINE__);
					}
					
					goto doMore;
				
					break;
			}
			break;
		
		case SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE:
			if (sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY)) {
				sdcard_deselect();
				
				sdcard.state = SDCARD_STATE_READY;
				
#ifdef SDCARD_PROFILING
                if (sdcard.profiler) {
                    sdcard.profiler(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                }
#endif
			} else if (millis() > sdcard.operationStartTime + SDCARD_TIMEOUT_WRITE_MSEC) {
				sdcard_reset();
				goto doMore;
			}
			break;
		
		case SDCARD_STATE_NOT_PRESENT:
//			printf("SDCARD_STATE_NOT_PRESENT!!\r\n");
		default:
			;
	}
	
	/* Is the card's initialisation taking too long? */
	if (sdcard.state >= SDCARD_STATE_RESET && sdcard.state < SDCARD_STATE_READY
		&& millis() - sdcard.operationStartTime > SDCARD_TIMEOUT_INIT_MILLIS) {
		sdcard_reset();
	}
	
	return sdcard_isReady();
}

/**
 * Begin writing a series of consecutive blocks beginning at the given block index. This will allow (but not require)
 * the SD card to pre-erase the number of blocks you specify, which can allow the writes to complete faster.
 *
 * Afterwards, just call sdcard_writeBlock() as normal to write those blocks consecutively.
 *
 * It's okay to abort the multi-block write at any time by writing to a non-consecutive address, or by performing a read.
 *
 * Returns:
 *		SDCARD_OPERATION_SUCCESS		- Multi-block write has been queued
 * 		SDCARD_OPERATION_BUSY			- The card is already busy and cannot accept your write
 *		SDCARD_OPERATION_FAILURE		- A fatal error occured, card will be reset
 */
sdcardOperationStatus_e sdcard_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount)
{
	if (sdcard.state != SDCARD_STATE_READY) {
		if (sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS) {
			if (blockIndex == sdcard.multiWriteNextBlock) {
				/* Assume that the caller wants to continue the multi-block write they already have in progress! */
				return SDCARD_OPERATION_SUCCESS;
			} else if (sdcard_endWriteBlocks() != SDCARD_OPERATION_SUCCESS) {
				return SDCARD_OPERATION_BUSY;
			} // else we've completed the previous multi-block write and can fall through to start the new one
		} else {
			/* state != multiple write blocks */
			return SDCARD_OPERATION_BUSY;
		}
	}
	
	sdcard_select();
	
	/**
	 * SDCARD_ACOMMAND_SET_WR_BLOCK_ERASE_COUNT = CMD23
	 * SDCARD_COMMAND_WRITE_MULTIPLE_BLOCK = CMD25
	 */
	if (
		sdcard_sendAppCommand(SDCARD_ACOMMAND_SET_WR_BLOCK_ERASE_COUNT, blockCount) == 0
	&& sdcard_sendCommand(SDCARD_COMMAND_WRITE_MULTIPLE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE) == 0
	) {
		sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
		sdcard.multiWriteBlocksRemain = blockCount;
		sdcard.multiWriteNextBlock = blockIndex;
		
		/* Leave the card selected as the process will keep going to the sdcard_poll() function with SDCARD_STATE_WRITING_MULTIPLE_BLOCKS state */
		return SDCARD_OPERATION_SUCCESS;
	} else {
		sdcard_deselect();
		
		sdcard_reset();
		
		return SDCARD_OPERATION_FAILURE;
	}
}

/**
 * Begin sending a buffer of SDCARD_BLOCK_SIZE (512) bytes to the SD card.
 */
static void sdcard_sendDataBlockBegin(uint8_t *buffer, bool multiBlockWrite)
{
	/* Card wants 8 dummy clock cycles between the write command's response and a data block beginning */
	spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
	
	spiTransferByte(SDCARD_SPI_INSTANCE, multiBlockWrite ? SDCARD_MULTIPLE_BLOCK_WRITE_START_TOKEN : SDCARD_SINGLE_BLOCK_WRITE_START_TOKEN);
	
	if (useDMAForTx) {
#ifdef SDCARD_DMA_CHANNEL_TX
#if defined(USE_HAL_DRIVER)
        sdDMAHandle = spiSetDMATransmit(SDCARD_DMA_CHANNEL_TX, SDCARD_DMA_CHANNEL, SDCARD_SPI_INSTANCE, buffer, SDCARD_BLOCK_SIZE);
#else
	/* Queue the transmission of the sector payload */
#ifdef SDCARD_DMA_CLK
		RCC_AHB1PeriphClockCmd(SDCARD_DMA_CLK, ENABLE);
#endif
		DMA_InitTypeDef DMA_InitStructure;
		
		DMA_StructInit(&DMA_InitStructure);
		
#ifdef SDCARD_DMA_CHANNEL
		DMA_InitStructure.DMA_Channel = SDCARD_DMA_CHANNEL;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
#else
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) buffer;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
#endif
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &SDCARD_SPI_INSTANCE->DR;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		
		DMA_InitStructure.DMA_BufferSize = SDCARD_BLOCK_SIZE;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		
		DMA_DeInit(SDCARD_DMA_CHANNEL_TX);
		DMA_Init(SDCARD_DMA_CHANNEL_TX, &DMA_InitStructure);
		
		DMA_Cmd(SDCARD_DMA_CHANNEL_TX, ENABLE);
		
		SPI_I2S_DMACmd(SDCARD_SPI_INSTANCE, SPI_I2S_DMAReq_Tx, ENABLE);
#endif
#endif
	} else {
		/* Send the first chunk now */
		spiTransfer(SDCARD_SPI_INSTANCE, NULL, buffer, SDCARD_NON_DMA_CHUNK_SIZE);
	}
}

/**
 * Write the 512-byte block from the given buffer into the block with the given index.
 *
 * If the write does not complete immediately, your callback will be called later. If the write was successful, the
 * buffer pointer will be the same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * Returns:
 *		SDCARD_OPERATION_IN_PROGRESS		- Your buffer is currently being transmitted to the card and your callback will be
 *											  called later to report the completion. The buffer pointer must remain valid until
 *											  that time.
 *		SDCARD_OPERATION_SUCCESS			- Your buffer has been transmitted to the card now.
 *		SDCARD_OPERATION_BUSY				- The card is already busy and cannot accept your write.
 *		SDCARD_OPERATION_FAILURE			- Your write was rejected by the card, card will be reset.
 */
sdcardOperationStatus_e sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
	uint8_t status;
	
#ifdef SDCARD_PROFILING
	sdcard.pendingOperation.profileStartTime = micros();
#endif
	
	doMore:
	switch (sdcard.state) {
		case SDCARD_STATE_WRITING_MULTIPLE_BLOCKS:
			/* Do we need to cancel the previous multi-block write? */
			if (blockIndex != sdcard.multiWriteNextBlock) {
				if (sdcard_endWriteBlocks() == SDCARD_OPERATION_SUCCESS) {
					/* Now we've entered the ready state, we can try again */
					goto doMore;
				} else {
					return SDCARD_OPERATION_BUSY;
				}
			}
			
			/* We are continuing a multi-block write */
			break;
		
		case SDCARD_STATE_READY:
			/* We are not continuing a multi-block write so we need to send a single-block write command */
			sdcard_select();
			
			/* Standard size cards use byte addressing, high capacity cards use block addressing */
			status = sdcard_sendCommand(SDCARD_COMMAND_WRITE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE);
			if (status != 0) {
				sdcard_deselect();
				
				sdcard_reset();
				
				return SDCARD_OPERATION_FAILURE;
			}
		
			break;
		
		default:
			return SDCARD_OPERATION_BUSY;
	}
	
	sdcard_sendDataBlockBegin(buffer, sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS);
	
	sdcard.pendingOperation.buffer = buffer;
	sdcard.pendingOperation.blockIndex = blockIndex;
	sdcard.pendingOperation.callback = callback;
	sdcard.pendingOperation.callbackData = callbackData;
	sdcard.pendingOperation.chunkIndex = 1;		// (for non-DMA transfers) we've sent chunk #0 already
	sdcard.state = SDCARD_STATE_SENDING_WRITE;
	
	return SDCARD_OPERATION_IN_PROGRESS;
}

/**
 * Read the 512-byte block with the given index into the given 512-byte buffer.
 * 
 * When the read completes, your callback will be called. If the read was successful, the buffer pointer will be the 
 * same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * You must keep the pointer to the buffer valid until the operation completes!
 *
 * Returns:
 * 		true - The operation was successfully queued for later completion, your callback will be called later.
 *		false - The operation could not be started due to the card being busy (try again later).
 */
bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
//	printf("sdcard.state: %u, %s, %d\r\n", sdcard.state, __FUNCTION__, __LINE__);	// sdcard.state = 4 (SDCARD_STATE_READY)
	
	/* Check if sdcard.state is SDCARD_STATE_READY (number 4) */
	if (sdcard.state != SDCARD_STATE_READY) {
		/* If sdcard.state is not read, check if sdcard is doing multiple write blocks operation */
		if (sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS) {
			/* If sdcard.state is in multiple write blocks state, check if the write operation is finished successfully or not */
			if (sdcard_endWriteBlocks() != SDCARD_OPERATION_SUCCESS) {
				/* if the multiple write operation is finished successfully, then fall through to read operation afterwards, 
 				 * otherwise, return false indicating that the read block operation is failed.
				 */
				return false;
			}
		} else {
			/* If sdcard.state is not ready and not in the multiple write blocks state either, then read operation failed, return false */
			return false;
		}
	}
	
	sdcard_select();
	
	/** Standard size cards use byte addressing, high capacity cards use block addressing
	 *
	 * 	SDCARD_COMMAND_READ_SINGLE_BLOCK = CMD17
	 */
	uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_READ_SINGLE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE);
	
	/* sending read command successfully */
	if (status == 0) {
		sdcard.pendingOperation.buffer = buffer;
		sdcard.pendingOperation.blockIndex = blockIndex;
		sdcard.pendingOperation.callback = callback;
		sdcard.pendingOperation.callbackData = callbackData;
		
		sdcard.state = SDCARD_STATE_READING;
		
		sdcard.operationStartTime = millis();			// read operation STARTING TIME
		
		/* Leave the card selected for the whole transaction, do not invoke the sdcard_deselect() function
		 * 
		 * INFO: SDCARD READ operation will be handled in the sdcard_poll() function thereafter
		 */
		
		return true;
	} else {
		/* read command failed  */
		sdcard_deselect();
		
		return false;
	}
}

#endif	// #ifdef USE_SDCARD
