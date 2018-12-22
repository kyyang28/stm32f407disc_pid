
#include <stdio.h>
#include <stdint.h>

#include "target.h"

#ifdef BLACKBOX

#include "feature.h"
#include "configMaster.h"
#include "system.h"
#include "blackbox_io.h"
#include "axis.h"
#include "mixer.h"

#define SLOW_FRAME_INTERVAL						4096

typedef struct blackboxMainState_s {
	uint32_t time;
	
	int32_t axisPID_P[XYZ_AXIS_COUNT], axisPID_I[XYZ_AXIS_COUNT], axisPID_D[XYZ_AXIS_COUNT];
	
	int16_t rcCommand[4];
	int16_t gyroADC[XYZ_AXIS_COUNT];
	int16_t accSmooth[XYZ_AXIS_COUNT];
	int16_t debug[4];
	int16_t motor[MAX_SUPPORTED_MOTORS];
	
	uint16_t vbatLastest;
	uint16_t amperageLatest;
	
	uint16_t rssi;
}blackboxMainState_t;

typedef enum BlackboxState {
	BLACKBOX_STATE_DISABLED = 0,				// 0
	BLACKBOX_STATE_STOPPED,						// 1
	BLACKBOX_STATE_PREPARE_LOG_FILE,			// 2
	BLACKBOX_STATE_SEND_HEADER,					// 3
	BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,		// 4
	BLACKBOX_STATE_SEND_GPS_H_HEADER,			// 5
	BLACKBOX_STATE_SEND_GPS_G_HEADER,			// 6
	BLACKBOX_STATE_SEND_SLOW_HEADER,			// 7
	BLACKBOX_STATE_SEND_SYSINFO,				// 8
	BLACKBOX_STATE_PAUSED,						// 9
	BLACKBOX_STATE_RUNNING,						// 10
	BLACKBOX_STATE_SHUTTING_DOWN				// 11
}BlackboxState;

#define BLACKBOX_FIRST_HEADER_SENDING_STATE			BLACKBOX_STATE_SEND_HEADER
#define BLACKBOX_LAST_HEADER_SENDING_STATE			BLACKBOX_STATE_SEND_SYSINFO

static struct {
	uint32_t headerIndex;
	
	/* Since these fields are used during different blackbox states (never simultaneously)
	 * we can overlap them to save on RAM.
	 */
	union {
		int fieldIndex;
		uint32_t startTime;
	}u;
}xmitState;

static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;

static bool blackboxLoggedAnyFrames;

static uint16_t blackboxSlowFrameIterationTimer;

/* Keep a history of length 2, plus a buffer for MW to store the new values into. */
static blackboxMainState_t blackboxHistoryRing[3];

/* These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old) */
static blackboxMainState_t *blackboxHistory[3];

/**
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs to encode.
 */
//static uint16_t vbatReference;

static int gcd(int num, int denom)
{
	if (denom == 0) {
		return num;
	}
	
	return gcd(denom, num % denom);
}

static void blackboxSetState(BlackboxState newState)
{
	/* Perform initial setup required for the new state */
	switch (newState) {
		case BLACKBOX_STATE_PREPARE_LOG_FILE:
			blackboxLoggedAnyFrames = false;
			break;
		
		case BLACKBOX_STATE_SEND_HEADER:
			blackboxHeaderBudget = 0;
			xmitState.headerIndex = 0;
			xmitState.u.startTime = millis();
			break;
		
		case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
		case BLACKBOX_STATE_SEND_GPS_G_HEADER:
		case BLACKBOX_STATE_SEND_GPS_H_HEADER:
		case BLACKBOX_STATE_SEND_SLOW_HEADER:
			xmitState.headerIndex = 0;
			xmitState.u.fieldIndex = -1;
			break;
		
		case BLACKBOX_STATE_SEND_SYSINFO:
			xmitState.headerIndex = 0;
			break;
		
		case BLACKBOX_STATE_RUNNING:
			/* Force a slow frame to be written on the first iteration */
			blackboxSlowFrameIterationTimer = SLOW_FRAME_INTERVAL;
			break;
		
		case BLACKBOX_STATE_SHUTTING_DOWN:
			xmitState.u.startTime = millis();
			break;
		
		default:
			;
	}
	
	blackboxState = newState;
}

void validateBlackboxConfig(void)
{
	int div;
	
	if (BlackboxConfig()->rate_num == 0 || BlackboxConfig()->rate_denom == 0
			|| BlackboxConfig()->rate_num >= BlackboxConfig()->rate_denom) {
		BlackboxConfig()->rate_num = 1;
		BlackboxConfig()->rate_denom = 1;
	} else {
		/**
		 * Reduce the fraction the user entered as much as possible (makes the recorded/skipped frame pattern repeat
		 * itself more frequently)
		 */
		div = gcd(BlackboxConfig()->rate_num, BlackboxConfig()->rate_denom);
		
		BlackboxConfig()->rate_num /= div;
		BlackboxConfig()->rate_denom /= div;
	}
	
	/* If we have chosen an unsupported device, change the device to serial */
	switch (BlackboxConfig()->device) {
#ifdef USE_FLASHFS
		case BLACKBOX_DEVICE_FLASH:
#endif

#ifdef USE_SDCARD
		case BLACKBOX_DEVICE_SDCARD:
#endif
		case BLACKBOX_DEVICE_SERIAL:
			/* Device supported, leave the setting alone */
			break;
		
		default:
			BlackboxConfig()->device = BLACKBOX_DEVICE_SERIAL;
	}
}

/**
 * Start Blackbox logging if it is not already running.
 * Intended to be called upon arming.
 *
 * This function only executes once after the quad is ARMed.
 */
void startBlackbox(void)
{
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
	
	if (blackboxState == BLACKBOX_STATE_STOPPED) {
		/* Validate blackbox configuration */
		validateBlackboxConfig();
		
		/* Check if blackbox device is open or not */
		if (!blackboxDeviceOpen()) {
			blackboxSetState(BLACKBOX_STATE_DISABLED);
			return;
		}
		
		/* Initialise history */
		blackboxHistory[0] = &blackboxHistoryRing[0];
		blackboxHistory[1] = &blackboxHistoryRing[1];
		blackboxHistory[2] = &blackboxHistoryRing[2];
		
//		vbatReference = vbatLatest;
		
		/* Build blackbox condition cache */
//		blackboxBuildConditionCache();
		
//        blackboxModeActivationConditionPresent = isModeActivationConditionPresent(modeActivationProfile()->modeActivationConditions, BOXBLACKBOX);

//        blackboxIteration = 0;
//        blackboxPFrameIndex = 0;
//        blackboxIFrameIndex = 0;

        /*
         * Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
         * it finally plays the beep for this arming event.
         */
//        blackboxLastArmingBeep = getArmingBeepTimeMicros();
//        blackboxLastFlightModeFlags = rcModeActivationMask; 		// record startup status		
		
		/* Set blackboxState to BLACKBOX_STATE_PREPARE_LOG_FILE (2) */
		blackboxSetState(BLACKBOX_STATE_PREPARE_LOG_FILE);
	}
}

static bool canUseBlackboxWithCurrentConfiguration(void)
{
	return feature(FEATURE_BLACKBOX) && 
		(BlackboxConfig()->device != BLACKBOX_SDCARD || feature(FEATURE_SDCARD));
}

void handleBlackbox(timeUs_t currentTimeUs)
{
//	int i;
	
	if (blackboxState >= BLACKBOX_FIRST_HEADER_SENDING_STATE && blackboxState <= BLACKBOX_LAST_HEADER_SENDING_STATE) {
		blackboxReplenishHeaderBudget();
	}
	
	switch (blackboxState) {
		case BLACKBOX_STATE_PREPARE_LOG_FILE:
			/* Keeps calling blackboxDeviceBeginLog() until the function returns true */
			if (blackboxDeviceBeginLog()) {
				blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
			}
			break;
		
		case BLACKBOX_STATE_SEND_HEADER:
//			printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
			/* On entry of this state, xmitState.headerIndex is 0 and startTime is initiallised */
		
			/**
		     * JUST FOR TESTING RIGHT NOW.
		     * Write a dummy contents to the SDCard and read it.
		     */
			blackboxPrint("Start of log\n");
			blackboxPrint("Yankun YANG\n");
			blackboxPrint("QUADYANG ");
			blackboxPrint("ABC\n");
			blackboxPrint("NFC\n");
			blackboxWrite('Y');
			blackboxWrite('\n');
			blackboxWrite('A');
			blackboxWrite('\n');
			blackboxWrite('N');
			blackboxWrite('\n');
			blackboxWrite('G');
			blackboxWrite('\n');

#if 0
			/**
			 * We can do flushing here, but the SDCard will flush each round anyway
			 */
			if (blackboxDeviceFlushForce()) {
//				printf("Flush the contents to SDCard successfully!\r\n");
			}
#endif
			
			/* TODO: Read SDCard */
//			const char recvBuff[50];
//			blackboxRead(recvBuff);
//			printf("recvBuff: %s, %s, %d\r\n", recvBuff, __FUNCTION__, __LINE__);
			
			/** blackboxStopLogging() calling afatfs_fclose(blackboxSDCard.logFile, NULL) to close the file handle just for testing
			 * otherwise, the programs will keep writing information into card
			 */
			blackboxStopLogging();	// JUST FOR TESTING, HOWEVER ALWAYS CLOSE FILE HANDLE WHENEVER WE ARE NOT WRITING CONTENTS TO THE SDCARD.
		
			/**
		     * Once the UART has had time to init, transmit the header in chunks so we don't overflow its transmit buffer,
		     * overflow the OpenLog's buffer, or keep the main loop busy for too long.
			 */
			
			break;
			
		default:
			break;
	}
}

void initBlackbox(void)
{
	if (canUseBlackboxWithCurrentConfiguration()) {
		blackboxSetState(BLACKBOX_STATE_STOPPED);
	} else {
		blackboxSetState(BLACKBOX_STATE_DISABLED);
	}
	
//	printf("blackboxState: %u, %s, %s, %d\r\n", blackboxState, __FILE__, __FUNCTION__, __LINE__);
}
#endif	// BLACKBOX
