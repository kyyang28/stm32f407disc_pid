#ifndef __FAT_STANDARD_H
#define __FAT_STANDARD_H

#include <stdint.h>
#include <stdbool.h>

#define FAT_FILENAME_LENGTH							11
#define FAT_SMALLEST_LEGAL_CLUSTER_NUMBER			2

#define FAT_DELETED_FILE_MARKER 					0xE5

#define MBR_PARTITION_TYPE_FAT16					0x06
#define MBR_PARTITION_TYPE_FAT32					0x0B
#define MBR_PARTITION_TYPE_FAT32_LBA				0x0C
#define MBR_PARTITION_TYPE_FAT16_LBA				0x0E

/* Signature bytes found at index 510 and 511 in the volume ID sector */
#define FAT_VOLUME_ID_SIGNATURE_1					0x55
#define FAT_VOLUME_ID_SIGNATURE_2					0xAA

#define FAT_DIRECTORY_ENTRY_SIZE					32
#define FAT_SMALLEST_LEGAL_CLUSTER_NUMBER			2

#define FAT_MAXIMUM_FILESIZE						0xFFFFFFFF

/* Use less and equal sign to determine the FAT type */
#define FAT12_MAX_CLUSTERS							4084		// official FAT doc (fatgen103.pdf) uses 4085 with less than, not less and equal
#define FAT16_MAX_CLUSTERS							65524		// official FAT doc (fatgen103.pdf) uses 65525 with less than, not less and equal

#define FAT_FILE_ATTRIBUTE_READ_ONLY				0x01
#define FAT_FILE_ATTRIBUTE_HIDDEN					0x02
#define FAT_FILE_ATTRIBUTE_SYSTEM					0x04
#define FAT_FILE_ATTRIBUTE_VOLUME_ID				0x08
#define FAT_FILE_ATTRIBUTE_DIRECTORY				0x10
#define FAT_FILE_ATTRIBUTE_ARCHIVE					0x20

/* According to spec <fatgen103.pdf>, p25 */
#define FAT_MAKE_DATE(year, month, day)     		(day | (month << 5) | ((year - 1980) << 9))
#define FAT_MAKE_TIME(hour, minute, second) 		((second / 2) | (minute << 5) | (hour << 11))

typedef enum {
	FAT_FILESYSTEM_TYPE_INVALID,			// 0
	FAT_FILESYSTEM_TYPE_FAT12,				// 1
	FAT_FILESYSTEM_TYPE_FAT16,				// 2
	FAT_FILESYSTEM_TYPE_FAT32				// 3
} fatFilesystemType_e;

typedef struct mbrPartitionEntry_t {
	uint8_t bootFlag;
	uint8_t chsBegin[3];
	uint8_t type;
	uint8_t chsEnd[3];
	uint32_t lbaBegin;
	uint32_t numSectors;
}__attribute__((packed)) mbrPartitionEntry_t;

typedef struct fat16Descriptor_t {
	uint8_t driveNumber;
	uint8_t reserved1;
	uint8_t bootSignature;
	uint32_t volumeID;
	char volumeLabel[11];
	char fileSystemType[8];
}__attribute__((packed)) fat16Descriptor_t;

typedef struct fat32Descriptor_t {
	uint32_t FATSize32;
	uint16_t extFlags;
	uint16_t fsVer;
	uint32_t rootCluster;
	uint16_t fsInfo;
	uint16_t backupBootSector;
	uint8_t reserved[12];
	uint8_t driveNumber;
	uint8_t reserved1;
	uint8_t bootSignature;
	uint32_t volumeID;
	char volumeLabel[11];
	char fileSystemType[8];
}__attribute__((packed)) fat32Descriptor_t;

typedef struct fatVolumeID_t {
	uint8_t jmpBoot[3];
	char oemName[8];
	uint16_t bytesPerSector;
	uint8_t sectorsPerCluster;
	uint16_t reservedSectorCount;
	uint8_t numFATs;
	uint16_t rootEntryCount;
	uint16_t totalSectors16;
	uint8_t media;
	uint16_t FATSize16;
	uint16_t sectorsPerTrack;
	uint16_t numHeads;
	uint32_t hiddenSectors;
	uint32_t totalSectors32;
	union {
		fat16Descriptor_t fat16;
		fat32Descriptor_t fat32;
	}__attribute__((packed)) fatDescriptor;		// add __attribute__((packed)) for KEIL IDE maybe
}__attribute__((packed)) fatVolumeID_t;

typedef struct fatDirectoryEntry_t {
	char filename[FAT_FILENAME_LENGTH];		// 11 bytes
	uint8_t attrib;							// 1 byte
	uint8_t ntReserved;						// 1 byte
	uint8_t creationTimeTenths;				// 1 byte
	uint16_t creationTime;					// 2 bytes
	uint16_t creationDate;					// 2 bytes
	uint16_t lastAccessDate;				// 2 bytes
	uint16_t firstClusterHigh;				// 2 bytes
	uint16_t lastWriteTime;					// 2 bytes
	uint16_t lastWriteDate;					// 2 bytes
	uint16_t firstClusterLow;				// 2 bytes
	uint32_t fileSize;						// 4 bytes
}__attribute__((packed)) fatDirectoryEntry_t;	// total: 11 + 1 + 1 + 1 + 2 + 2 + 2 + 2 + 2 + 2 + 2 + 4 = 32

bool fat_isDirectoryEntryTerminator(fatDirectoryEntry_t *entry);
bool fat_isDirectoryEntryEmpty(fatDirectoryEntry_t *entry);
bool fat_isFreeSpace(uint32_t clusterNumber);

uint32_t fat32_decodeClusterNumber(uint32_t clusterNumber);
bool fat16_isEndOfChainMarker(uint16_t clusterNumber);
bool fat32_isEndOfChainMarker(uint32_t clusterNumber);
void fat_convertFilenameToFATStyle(const char *filename, uint8_t *fatFilename);

#endif	// __FAT_STANDARD_H
