
#include <stdio.h>
#include <string.h>

#include "target.h"
#include "configMaster.h"
#include "blackbox_io.h"
#include "asyncfatfs.h"
#include "maths.h"

/* How many bytes can we transmit per loop iteration when writing headers? */
static uint8_t blackboxMaxHeaderBytesPerIteration;

/* How many bytes can we write *this* iteration without overflowing transmit buffers or overstressing the OpenLog? */
int32_t blackboxHeaderBudget;

#ifdef USE_SDCARD

#define LOGFILE_PREFIX						"LOG"
#define LOGFILE_SUFFIX						"txt"
//#define LOGFILE_SUFFIX						"BFL"

static struct {
	afatfsFilePtr_t logFile;
	afatfsFilePtr_t logDirectory;
	afatfsFinder_t logDirectoryFinder;
	uint32_t largestLogFileNumber;
	
	enum {
		BLACKBOX_SDCARD_INITIAL,						// 0
		BLACKBOX_SDCARD_WAITING,						// 1
		BLACKBOX_SDCARD_ENUMERATE_FILES,				// 2
		BLACKBOX_SDCARD_CHANGE_INTO_LOG_DIRECTORY,		// 3
		BLACKBOX_SDCARD_READY_TO_CREATE_LOG,			// 4
		BLACKBOX_SDCARD_READY_TO_LOG					// 5
	} state;
} blackboxSDCard;

#endif

/**
 * Attempt to open the logging device.
 *
 * Returns true if successful.
 */
bool blackboxDeviceOpen(void)
{
	switch (BlackboxConfig()->device) {
		case BLACKBOX_DEVICE_SERIAL:
			break;
		
#ifdef USE_FLASHFS
		case BLACKBOX_DEVICE_FLASH:
			break;
#endif
		
#ifdef USE_SDCARD
		case BLACKBOX_DEVICE_SDCARD:
			if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_FATAL || afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_UNKNOWN || afatfs_isFull()) {
				return false;
			}
			
			blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;
			
			return true;
//			break;
#endif
		default:
			return false;
	}
	
	return false;
}

void blackboxWrite(uint8_t value)
{
	switch (BlackboxConfig()->device) {
#ifdef USE_FLASHFS
		case BLACKBOX_DEVICE_FLASH:
			break;
#endif
		
#ifdef USE_SDCARD
		case BLACKBOX_DEVICE_SDCARD:
			afatfs_fputc(blackboxSDCard.logFile, value);
			break;
#endif
		
		case BLACKBOX_DEVICE_SERIAL:
//			serialWrite(blackboxPort, value);	// TODO: if necessary
			break;
	}
}

void blackboxRead(const char *recvBuffer)
{
	switch (BlackboxConfig()->device) {
#ifdef USE_FLASHFS
		case BLACKBOX_DEVICE_FLASH:
			break;
#endif
		
#ifdef USE_SDCARD
		case BLACKBOX_DEVICE_SDCARD:
			afatfs_fread(blackboxSDCard.logFile, (uint8_t *)recvBuffer, strlen(recvBuffer));
			break;
#endif
		
		case BLACKBOX_DEVICE_SERIAL:
			break;
	}
}

/**
 * If there is data waiting to be written to the blackbox device, attempt to write (a portion of) that now.
 *
 * Returns true if all data has been written to the device.
 */
bool blackboxDeviceFlushForce(void)
{
	switch (BlackboxConfig()->device) {
		case BLACKBOX_DEVICE_SERIAL:
			/* Nothing need to speed up flushing on serial, as serial is continuously being drained out of its buffer */
//			return isSerialTransmitBufferEmpty(blackboxPort);
		
#ifdef USE_FLASHFS
		case BLACKBOX_DEVICE_FLASH:
//			return flashfsFlushAsync();
#endif
		
#ifdef USE_SDCARD
		case BLACKBOX_DEVICE_SDCARD:
			/**
		     * SD card will flush itself without us calling it, but we need to call flush manually in order to check
		     * if it's done yet or not!
		     */
			return afatfs_flush();
#endif
		
		default:
			return false;
	}
}

/**
 * Call once every loop iteration in order to maintain the global blackboxHeaderBudget with the number of bytes we can transmit this iteration.
 */
void blackboxReplenishHeaderBudget(void)
{
	int32_t freeSpace;
	
	switch (BlackboxConfig()->device) {
		case BLACKBOX_DEVICE_SERIAL:
//			freeSpace = serialTxBytesFree(blackboxPort);
			break;
		
#ifdef USE_FLASHFS
		case BLACKBOX_DEVICE_FLASH:
//			freeSpace = flashfsGetWriteBufferFreeSpace();
			break;
#endif
		
#ifdef USE_SDCARD
		case BLACKBOX_DEVICE_SDCARD:
			freeSpace = afatfs_getFreeBufferSpace();
			break;
#endif
		default:
			freeSpace = 0;
	}
	
	/* blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION (64) */
	blackboxHeaderBudget = MIN(MIN(freeSpace, blackboxHeaderBudget + blackboxMaxHeaderBytesPerIteration), BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET);
}

#ifdef USE_SDCARD

/**
 * Callback function to create the log directory.
 *
 * Assign the opened directory handle pointer to the blackboxSDCard.logDirectory handle pointer.
 */
static void blackboxLogDirCreated(afatfsFilePtr_t directory)
{
//	printf("directory addr: 0x%x, %s, %d\r\n", (uint32_t)directory, __FUNCTION__, __LINE__);	// directory addr: 0x20002a34
	
	if (directory) {
		blackboxSDCard.logDirectory = directory;
				
//		printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
				
		afatfs_findFirst(blackboxSDCard.logDirectory, &blackboxSDCard.logDirectoryFinder);
		
		blackboxSDCard.state = BLACKBOX_SDCARD_ENUMERATE_FILES;
	} else {
		/* Retry */
		blackboxSDCard.state = BLACKBOX_SDCARD_INITIAL;
	}
}

/**
 * Callback function to create the the file.
 *
 * Assign the opened file handle pointer to the blackboxSDCard.logFile handle pointer.
 */
static void blackboxLogFileCreated(afatfsFilePtr_t file)
{
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);

#if 1
	if (file) {
		blackboxSDCard.logFile = file;
		
		blackboxSDCard.largestLogFileNumber++;
		
		blackboxSDCard.state = BLACKBOX_SDCARD_READY_TO_LOG;
		printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
	} else {
		/* Retry */
		blackboxSDCard.state = BLACKBOX_SDCARD_READY_TO_CREATE_LOG;
//		printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
	}
#endif	
}

/**
 * Function to create the log file.
 */
static void blackboxCreateLogFile(void)
{
//	printf("blackboxSDCard.largetstLogFileNumber: %u, %s, %s, %d\r\n",blackboxSDCard.largetstLogFileNumber, __FILE__, __FUNCTION__, __LINE__);
	uint32_t remainder = blackboxSDCard.largestLogFileNumber + 1;
	
	char filename[] = LOGFILE_PREFIX "00000." LOGFILE_SUFFIX;
	
//	printf("filename: %s, %s, %s, %d\r\n",filename, __FILE__, __FUNCTION__, __LINE__);		// LOG00000.BFL
	
	/* Assigning the LOG number */
	for (int i = 7; i >= 3; i--) {
		filename[i] = (remainder % 10) + '0';
		remainder /= 10;
	}
	
	blackboxSDCard.state = BLACKBOX_SDCARD_WAITING;

//	printf("filename: %s, %s, %s, %d\r\n", filename, __FILE__, __FUNCTION__, __LINE__);		// LOG00001.BFL
	
	if (!afatfs_fopen(filename, "as", blackboxLogFileCreated)) {
		printf("Failed to open file: %s\r\n", filename);
	}
}

/**
 * Begin a new log on the SDCard.
 *
 * Keep calling until the function returns true (open is complete).
 */
static bool blackboxSDCardBeginLog(void)
{
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
	fatDirectoryEntry_t *directoryEntry;
	
	doMore:
	
	/**
	 * NOTE: According to AFATFS_MAX_OPEN_FILES (3), there are MAXIMUM 3 folders and/or files that can be created on the SDCard.
	 * So to create more FOLDERs and/or FILEs, we need to call afatfs_fclose() function to FREE the afatfs.openFiles[1-3].type to AFATFS_FILE_TYPE_NONE.
	 */
	switch (blackboxSDCard.state) {
		case BLACKBOX_SDCARD_INITIAL:
//			printf("afatfs_getFilesystemState(): %u, %s, %s, %d\r\n", afatfs_getFilesystemState(), __FILE__, __FUNCTION__, __LINE__);
			if (afatfs_getFilesystemState() == AFATFS_FILESYSTEM_STATE_READY) {
//				printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
				blackboxSDCard.state = BLACKBOX_SDCARD_WAITING;
				
				/* Create "logs" directory(folder) on the SDCard
				 * 
				 * TODO: HERE we can use RTC time and date as the directory name instead of "logs".
				 * Merge RTC programs later
				 */
				afatfs_mkdir("logs", blackboxLogDirCreated);

//				afatfs_mkdir("quadLogs", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("pwcLogs", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("yangLogs", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("yangLogs2", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("yangLogs3", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("yangLogs4", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("yangLogs5", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("yangLogs6", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("yangLogs7", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("yangLogs8", blackboxLogDirCreated);	// just for testing
//				afatfs_mkdir("yangLogs9", blackboxLogDirCreated);	// just for testing
//				blackboxCreateLogFile();					// LOG000001.BFL
//				blackboxCreateLogFile();					// LOG000002.BFL
//				blackboxCreateLogFile();					// LOG000003.BFL
//				blackboxCreateLogFile();					// LOG000003.BFL
//				blackboxCreateLogFile();					// LOG000003.BFL

//				blackboxSDCard.state = BLACKBOX_SDCARD_READY_TO_LOG;

#if 0
				for (int i = 0; i < 8; i++) {
					printf("largestNumber: %u\r\n", blackboxSDCard.largestLogFileNumber);
					uint32_t remainder = blackboxSDCard.largestLogFileNumber + 1;
					
					char filename[] = LOGFILE_PREFIX "00000." LOGFILE_SUFFIX;
					
					//	printf("filename: %s, %s, %s, %d\r\n", filename, __FILE__, __FUNCTION__, __LINE__);		// LOG00000.BFL
					
					/* Assigning the LOG number */
					for (int i = 7; i >= 3; i--) {
						filename[i] = (remainder % 10) + '0';
						remainder /= 10;
					}
					
					/* Create a new file ONLY if it doesn't exist on the SDCard */
					if (!afatfs_fopen(filename, "as", blackboxLogFileCreated)) {		// JUST FOR TESTING
						printf("Failed to open file!!\r\n");
					}
					
					/**
					 * The program ONLY ALLOWS to create or open up to 3 maximum files. In order to create or open more files,
					 * it requires to close the file handle if the file does not need to be written or read. TO DO SO, 
					 * set the file->type to AFATFS_FILE_TYPE_NONE, which free the file handle in order to create a new file.
					 * 
					 * we can call afatfs_fclose() function when disarming the quadcopter (calling mwDisarm() function) or 
					 * when the joystick of the powered wheelchair doesn't move for 10 seconds (changeable).
					 */
					afatfs_fclose(blackboxSDCard.logFile, NULL);						// JUST FOR TESTING
				}
#endif
				
				/*
					file->attrib: 5
					file->type: 1
					afatfs.openFiles[0].type: 0
					file->attrib: 16
					file->type: 3
					afatfs.openFiles[0].type: 3
					afatfs.openFiles[1].type: 0
					fileMode: 0x3c, src\quad\io\asyncfatfs\asyncfatfs.c, afatfs_fopen, 3642
					file->attrib: 32
					file->type: 1
					src\quad\blackbox\blackbox_io.c, blackboxLogFileCreated, 136
					src\quad\blackbox\blackbox_io.c, blackboxSDCardBeginLog, 258
				*/
//				if (!afatfs_fopen("LOG00001.YQL", "as", blackboxLogFileCreated)) {
//					printf("Failed to open file!!\r\n");
//				}
			}
			break;
		
		case BLACKBOX_SDCARD_WAITING:
			/* Waiting for directory entry to be created */
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			break;
		
		case BLACKBOX_SDCARD_ENUMERATE_FILES:		// list all the files we need to create on the SDCard.
			while (afatfs_findNext(blackboxSDCard.logDirectory, &blackboxSDCard.logDirectoryFinder, &directoryEntry) == AFATFS_OPERATION_SUCCESS) {
				if (directoryEntry && !fat_isDirectoryEntryTerminator(directoryEntry)) {
//					printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
					/* If this is a log file, parse the log number from the filename */
					if (strncmp(directoryEntry->filename, LOGFILE_PREFIX, strlen(LOGFILE_PREFIX)) == 0
						&& strncmp(directoryEntry->filename + 8, LOGFILE_SUFFIX, strlen(LOGFILE_SUFFIX)) == 0) {
						char logSequenceNumberString[6];
							
						memcpy(logSequenceNumberString, directoryEntry->filename + 3, 5);
						logSequenceNumberString[5] = '\0';
//						printf("logSequenceNumberString: %s\r\n", logSequenceNumberString);
						
						blackboxSDCard.largestLogFileNumber = MAX((uint32_t) atoi(logSequenceNumberString), blackboxSDCard.largestLogFileNumber);
					}
				} else {
					/* We are done checking all the files on the card, now we can create a new log file */
					afatfs_findLast(blackboxSDCard.logDirectory);
					
					blackboxSDCard.state = BLACKBOX_SDCARD_CHANGE_INTO_LOG_DIRECTORY;
//					printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
					goto doMore;
				}
			}
			break;
		
		case BLACKBOX_SDCARD_CHANGE_INTO_LOG_DIRECTORY:
			/* Change into the log directory */
			if (afatfs_chdir(blackboxSDCard.logDirectory)) {
				/* We no longer need our open handle on the log directory */
				afatfs_fclose(blackboxSDCard.logDirectory, NULL);
				blackboxSDCard.logDirectory = NULL;

//				printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
								
				blackboxSDCard.state = BLACKBOX_SDCARD_READY_TO_CREATE_LOG;
				
//				printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
				
				goto doMore;
			}
			break;
		
		case BLACKBOX_SDCARD_READY_TO_CREATE_LOG:
//			printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
			blackboxCreateLogFile();
			break;
		
		case BLACKBOX_SDCARD_READY_TO_LOG:
			/* Log has been created!! */
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			return true;			// Returns true if log file has been created under LOGS directory
	}
	
	return false;	// Not finished init yet.
}

#endif	// USE_SDCARD

/**
 * Begin a new log (for devices which support separations between the logs of multiple flights).
 *
 * Keep calling until the function returns true (open is complete).
 */
bool blackboxDeviceBeginLog(void)
{
	switch (BlackboxConfig()->device) {
#ifdef USE_SDCARD
		case BLACKBOX_DEVICE_SDCARD:
			return blackboxSDCardBeginLog();
#endif
		default:
			return true;
	}
}

int blackboxPrint(const char *s)
{
	int length;
	
	switch (BlackboxConfig()->device) {
#ifdef USE_FLASHFS
		case BLACKBOX_DEVICE_FLASH:
			break;
#endif
		
#ifdef USE_SDCARD
		case BLACKBOX_DEVICE_SDCARD:
			length = strlen(s);
			afatfs_fwrite(blackboxSDCard.logFile, (const uint8_t *)s, length);
			break;
#endif
	}
    
    return 0;
}

/**
 * Helper function just for testing logging purposes.
 *
 * Calling afatfs_fclose() function to test whether 'Y' character (from calling blackboxWrite('Y')) is written to the SD Card 
 */
bool blackboxStopLogging(void)
{
	return afatfs_fclose(blackboxSDCard.logFile, NULL);
}
