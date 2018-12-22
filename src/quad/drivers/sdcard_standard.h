#ifndef __SDCARD_STANDARD_H
#define __SDCARD_STANDARD_H

#include <stdint.h>

typedef struct sdcardCSD_t {
	uint8_t data[16];
}sdcardCSD_t;

#define SDCARD_GET_CSD_FIELD(csd, version, fieldname) \
	readBitfield(csd.data, SDCARD_CSD_V ## version ## _ ## fieldname ## _OFFSET, SDCARD_CSD_V ## version ## _ ## fieldname ## _LEN)

/* For v1 and Standard Capacity cards */
#define SDCARD_CSD_V1_CSD_STRUCTURE_VER_OFFSET		0
#define SDCARD_CSD_V1_CSD_STRUCTURE_VER_LEN			2

#define SDCARD_CSD_V1_READ_BLOCK_LEN_OFFSET         44
#define SDCARD_CSD_V1_READ_BLOCK_LEN_LEN            4

#define SDCARD_CSD_V1_CSIZE_OFFSET                  54
#define SDCARD_CSD_V1_CSIZE_LEN                     12

#define SDCARD_CSD_V1_CSIZE_MULT_OFFSET             78
#define SDCARD_CSD_V1_CSIZE_MULT_LEN                3

#define SDCARD_CSD_V1_TRAILER_OFFSET				127
#define SDCARD_CSD_V1_TRAILER_LEN					1

/* For V2 high capacity cards */
#define SDCARD_CSD_V2_CSIZE_OFFSET                  58
#define SDCARD_CSD_V2_CSIZE_LEN                     22

#define SDCARD_SINGLE_BLOCK_READ_START_TOKEN		0xFE
#define SDCARD_SINGLE_BLOCK_WRITE_START_TOKEN		0xFE
#define SDCARD_MULTIPLE_BLOCK_WRITE_START_TOKEN		0xFC
#define SDCARD_MULTIPLE_BLOCK_WRITE_STOP_TOKEN		0xFD

#define SDCARD_BLOCK_SIZE							512

#define SDCARD_VOLTAGE_ACCEPTED_2_7_to_3_6			0x01

#define SDCARD_CSD_STRUCTURE_VERSION_1				0
#define SDCARD_CSD_STRUCTURE_VERSION_2				1

/* Index of R1 response format
 * Idle bit is set to 1 only when idle during initialisation phase
 */
#define SDCARD_R1_STATUS_BIT_IDLE					1
#define SDCARD_R1_STATUS_BIT_ERASE_RESET			2
#define SDCARD_R1_STATUS_BIT_ILLEGAL_COMMAND		4
#define SDCARD_R1_STATUS_BIT_COM_CRC_ERROR			8
#define SDCARD_R1_STATUS_BIT_ERASE_SEQUENCE_ERROR	16
#define SDCARD_R1_STATUS_BIT_ADDRESS_ERROR			32
#define SDCARD_R1_STATUS_BIT_PARAMETER_ERROR		64

#define SDCARD_COMMAND_GO_IDLE_STATE				0
#define SDCARD_COMMAND_SEND_OP_COND					1
#define SDCARD_COMMAND_SEND_IF_COND					8
#define SDCARD_COMMAND_SEND_CSD						9
#define SDCARD_COMMAND_SEND_CID						10
#define SDCARD_COMMAND_STOP_TRANSMISSION			12
#define SDCARD_COMMAND_SEND_STATUS					13
#define SDCARD_COMMAND_SET_BLOCKLEN					16
#define SDCARD_COMMAND_READ_SINGLE_BLOCK			17
#define SDCARD_COMMAND_READ_MULTIPLE_BLOCK			18
#define SDCARD_COMMAND_WRITE_BLOCK					24
#define SDCARD_COMMAND_WRITE_MULTIPLE_BLOCK			25
#define SDCARD_COMMAND_APP_CMD						55
#define SDCARD_COMMAND_READ_OCR						58

#define SDCARD_ACOMMAND_SEND_OP_COND				41
#define SDCARD_ACOMMAND_SET_WR_BLOCK_ERASE_COUNT	23

/* There are worst-case timeouts defined for High Speed Cards */
#define SDCARD_TIMEOUT_READ_MSEC					100
#define SDCARD_TIMEOUT_WRITE_MSEC					250

uint32_t readBitfield(uint8_t *buffer, unsigned bitIndex, unsigned bitLen);

#endif	// __SDCARD_STANDARD_H
