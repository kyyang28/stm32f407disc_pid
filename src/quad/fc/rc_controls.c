
#include <stdio.h>

#include "rc_controls.h"
#include "fc_core.h"
#include "runtime_config.h"
#include "rx.h"				// rcData[]
#include "maths.h"			// constrain
#include "sound_beeper.h"
#include "gyro.h"

static motorConfig_t *motorConfig;
static pidProfile_t *pidProfile;

int16_t rcCommand[4];			// interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

/* One bit per mode defined in boxId_e */
uint32_t rcModeActivationMask;

/* true if arming is done via the sticks (as opposed to a switch) */
static bool isUsingSticksToArm = true;

bool isAntiGravityModeActive(void)
{
	return (IS_RC_MODE_ACTIVE(BOXANTIGRAVITY) || feature(FEATURE_ANTI_GRAVITY));
}

bool isAirModeActive(void)
{
	return (IS_RC_MODE_ACTIVE(BOXAIRMODE) || feature(FEATURE_AIRMODE));
}

bool isUsingSticksForArming(void)
{
	return isUsingSticksToArm;
}

bool isModeActivationConditionPresent(modeActivationCondition_t *modeActivationConditions, boxId_e modeId)
{
	uint8_t index;
	
	for (index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
		modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];
		
		if (modeActivationCondition->modeId == modeId && IS_RANGE_USABLE(&modeActivationCondition->range)) {
			return true;
		}
	}
	
	return false;
}

bool isRangeActive(uint8_t auxChannelIndex, channelRange_t *range)
{
	if (!IS_RANGE_USABLE(range)) {
		return false;
	}
	
//	printf("auxChannelIndex: %u\r\n", auxChannelIndex);
//	printf("auxChannelIndex: %u\r\n", auxChannelIndex + NON_AUX_CHANNEL_COUNT);
//	printf("rcData[%d]: %u\r\n", auxChannelIndex + NON_AUX_CHANNEL_COUNT, rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT]);
	
	uint16_t channelValue = constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
	
	/*
	 * For example:  range->startStep = (900 - 900) / 25 = 0
	 *				 range->endStep = (1150 - 900) / 25 = 250 / 25 = 10
	 * channelValue = 987 >= 900 + (range->startStep * 25) ==> channelValue = 987 >= 900 + (0 * 25) ==> channelValue = 987 >= 900 is TRUE
	 * channelValue = 987 < 900 + (range->endStep * 25) ==> channelValue = 987 < 900 + (10 * 25) ==> channelValue = 987 < 1150 is TRUE
	 */
	return (channelValue >= 900 + (range->startStep * 25) && channelValue < 900 + (range->endStep * 25));
}

/** Update activated modes (BOXARM, BOXAIRMODE and so on)
 *
 */
void updateActivatedModes(modeActivationCondition_t *modeActivationConditions)
{
	rcModeActivationMask = 0;
	
	for (uint8_t index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
		modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];

//		printf("Index: %u\r\n", modeActivationCondition->auxChannelIndex);
//		printf("modeId: %d\r\n", modeActivationCondition->modeId);
//		printf("startStep: %u\r\n", modeActivationCondition->range.startStep);
//		printf("endStep: %u\r\n", modeActivationCondition->range.endStep);
		if (isRangeActive(modeActivationCondition->auxChannelIndex, &modeActivationCondition->range)) {
//			printf("modeId: %d\r\n", modeActivationCondition->modeId);
			ACTIVATE_RC_MODE(modeActivationCondition->modeId);

//			if (IS_RC_MODE_ACTIVE(BOXARM)) {
//				printf("Motors are ARMed!\r\n");
//			}
//						
//			if (IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
//				printf("Buzzer is ON!\r\n");
//				BEEP_ON;
//			}
//			
//			if (!IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
//				BEEP_OFF;
//			}
//			
//			if (IS_RC_MODE_ACTIVE(BOXANGLE)) {
//				printf("Angle mode is ON!\r\n");
//			}

//			if (IS_RC_MODE_ACTIVE(BOXHORIZON)) {
//				printf("HORIZON mode is ON!\r\n");
//			}
//			
////			if (!IS_RC_MODE_ACTIVE(BOXANGLE) && !IS_RC_MODE_ACTIVE(BOXHORIZON)) {
////				printf("MANUAL mode is ON!\r\n");
////			}
//			
//			if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
//				printf("AIRMODE is ON!\r\n");
//			}

//			printf("rcModeActivationMask: %u\r\n", rcModeActivationMask);
		}
	}
}

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, motorConfig_t *motorConfigToUse, pidProfile_t *pidProfileToUse)
{
	motorConfig = motorConfigToUse;
	pidProfile = pidProfileToUse;
	
	isUsingSticksToArm = !isModeActivationConditionPresent(modeActivationConditions, BOXARM);
}

void processRcStickPositions(rxConfig_t *rxConfig, throttleStatus_e throttleStatus, bool disarm_kill_switch)
{
	uint8_t tmp = 0;
	
	/* this hold sticks position for command combos. */
	static uint8_t rcSticks;
	
	/* this indicates the number of time (multiple of RC measurement at 50 HZ), the sticks must be maintained to run or switch off motors */
	static uint8_t rcDelayCommand;
	
	/* this is an extra guard for disarming through switch to prevent that one frame can disarm it */
	static uint8_t rcDisarmTick;
	
//	printf("armingFlags: %u, %s, %d\r\n", armingFlags, __FUNCTION__, __LINE__);
//	printf("mincheck: %u\r\n", rxConfig->mincheck);
//	printf("maxcheck: %u\r\n", rxConfig->maxcheck);
	
	/* +----------------------- STICKS COMMAND HANDLER -----------------------+ */
	/* Checking sticks positions for stick command combos using ROLL, PITCH, YAW and THROTTLE stick values
	 *
	 * ROL_LO			1 << 0		(01 << 0)
	 * ROL_CE			3 << 0		(11 << 0)
	 * ROL_HI			2 << 0		(10 << 0)
	 * 
	 * PIT_LO			1 << 2		(01 << 2)
	 * PIT_CE			3 << 2		(11 << 2)
	 * PIT_HI			2 << 2		(10 << 2)
	 *
	 * YAW_LO			1 << 4		(01 << 4)
	 * YAW_CE			3 << 4		(11 << 4)
	 * YAW_HI			2 << 4		(10 << 4)
	 *
	 * THR_LO			1 << 6		(01 << 6)
	 * THR_CE			3 << 6		(11 << 6)
	 * THR_HI			2 << 6		(10 << 6)
	 *
	 * Each RC channel occupies 2-bit of the whole 8-bit
	 * 
	 * Bits [7:6] THROTTLE positions
	 * 				00: Reserved
	 *				01: Throttle LOW
	 *				10: Throttle HIGH
	 *				11: Throttle CENTRE
	 *
	 * Bits [5:4] YAW positions
	 * 				00: Reserved
	 *				01: Yaw LOW
	 *				10: Yaw HIGH
	 *				11: Yaw CENTRE
	 *
	 * Bits [3:2] PITCH positions
	 * 				00: Reserved
	 *				01: Pitch LOW
	 *				10: Pitch HIGH
	 *				11: Pitch CENTRE
	 *
	 * Bits [1:0] ROLL positions
	 * 				00: Reserved
	 *				01: Roll LOW
	 *				10: Roll HIGH
	 *				11: Roll CENTRE
	 *
	 *     THR       YAW       PIT       ROL
	 * -----------------------------------------
	 * |  0 |  1 |  0 |  1 |  1	|  1 |  1 |  1 |
	 * -----------------------------------------
	 *
	 * For example:
	 * THR_LO + YAW_LO + ROL_CE + PIT_CE = (1 << 6 | 1 << 4 | 3 << 2 | 3 << 0) = (01011111)_2 = (95)_10
	 */
	for (int i = 0; i < 4; i++) {
		tmp >>= 2;
		
		/*
		 * rcData[0]: ROLL
		 * rcData[1]: PITCH
		 * rcData[2]: YAW
		 * rcData[3]: THROTTLE
		 *
		 */
//		printf("rcData[%d]: %u\r\n", i, rcData[i]);
		
		if (rcData[i] > rxConfig->mincheck) {
			/* Check for MIN */
			tmp |= 0x80;
		}
		
		if (rcData[i] < rxConfig->maxcheck) {
			/* Check for MAX */
			tmp |= 0x40;
		}
	}
	
	/* Check if the current sticks combo (tmp) is equal to the previous sticks combo (rcSticks)
	 * If they are equal, meaning the sticks combo are not changing (all the sticks are not moving significantly)
	 * then there is a rcDelay time till 250 * the looptime of rx task (50 Hz)
	 */
	if (tmp == rcSticks) {
		if (rcDelayCommand < 250)
			rcDelayCommand++;
	} else {
		/* If the current sticks combo (tmp) is NOT equal to the previous sticks combo (rcSticks), meaning the sticks have been moved.
		 * reset the rcDelayCommand (rc delay time elapse), ready for the next delay count up.
		 */
		rcDelayCommand = 0;
	}
	
	/* Assign the current sticks combos (tmp) to rcSticks */
	rcSticks = tmp;
//	printf("rcSticks: %u\r\n", rcSticks);
//	printf("rcDelayCommand: %u\r\n", rcDelayCommand);
//	printf("stickARM: %u\r\n", THR_LO + YAW_LO + ROL_CE + PIT_CE);		// THR_LO (1<<6) + YAW_LO(1<<4) + ROL_CE(3<<0) + PIT_CE(3<<2) = 95 (01011111)
	
	/* Perform actions */
	if (!isUsingSticksToArm) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);

		if (IS_RC_MODE_ACTIVE(BOXARM)) {
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			rcDisarmTick = 0;
			
			/* Arming via ARM BOX */
			if (throttleStatus == THROTTLE_LOW) {
				if (CHECK_ARMING_FLAG(OK_TO_ARM)) {
					mwArm();
//					printf("ARMED: %u, %s, %d\r\n", CHECK_ARMING_FLAG(ARMED), __FUNCTION__, __LINE__);
				}				
			}
		} else {
			/* Disarming via ARM BOX */
			if (CHECK_ARMING_FLAG(ARMED) && rxIsReceivingSignal()) {
//			if (CHECK_ARMING_FLAG(ARMED) && rxIsReceivingSignal() && !failsafeIsActive()) {
//				printf("%s, %d\r\n", __FUNCTION__, __LINE__);
				rcDisarmTick++;
//				printf("rcDisarmTick: %u\r\n", rcDisarmTick);
				if (rcDisarmTick > 3) {
					/* disarm_kill_switch = 1 */
					if (disarm_kill_switch) {
						mwDisarm();
//						printf("DISARMED: %u, %s, %d\r\n", CHECK_ARMING_FLAG(ARMED), __FUNCTION__, __LINE__);
					} else if (throttleStatus == THROTTLE_LOW) {
						mwDisarm();
//						printf("DISARMED: %u, %s, %d\r\n", CHECK_ARMING_FLAG(ARMED), __FUNCTION__, __LINE__);
					}
				}
			}
		}
	}
	
//	printf("rcDelayCmd: %u\r\n", rcDelayCommand);		// rcDelayCommand = 250 when sticks are not moving around
	
	if (rcDelayCommand != 20) {
		return;
	}
	
	/* Handle stick arming case */
	if (isUsingSticksToArm) {
		/* TODO: implement later if necessary */
	}
	
	if (CHECK_ARMING_FLAG(ARMED)) {
		return;
	}
	
	/* +------------------- Actions during not Armed -------------------+ */
	/**
	 *	Gyro Re-Calibration process: Throttle BOTTOM + Yaw to the LEFT + Pitch BOTTOM + Roll CENTRE
	 *
	 * THR_LO = 1 << 6	 	(01000000)
	 * YAW_LO = 1 << 4		(00010000)
	 * PIT_LO = 1 << 2		(00000100)
	 * ROL_CE = 3 << 0		(00000011)
	 *
	 * THR_LO + YAW_LO + PIT_LO + ROL_CE = (1 << 6 | 1 << 4 | 1 << 2 | 3 << 0)
	 *									 = (01010111)
	 *									 = 87
	 */
	if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
		/* Perform gyro calibration */
//		printf("gyroCali: %u\r\n", rcSticks);
		
		/* calibratingG = gyroCalculateCalibratingCycles() = (CALIBRATING_GYRO_CYCLES / gyro.targetLooptime) * CALIBRATING_GYRO_CYCLES
		 *												   = (1000 / 125) * 1000
		 *												   = (8 * 1000)
		 *												   = 8000
		 */
		gyroSetCalibrationCycles();

#ifdef GPS
		/* TODO: implementation later */
		if (feature(FEATURE_GPS)) {
			GPS_reset_home_position();
		}
#endif
	
#ifdef BARO
		/* TODO: implementation later */
		if (sensors(SENSOR_BARO)) {
			baroSetCalibrationCycles(10);	// calibrate barometer to new ground level (10 * 25 ms = ~250 ms non-blocking)
		}
#endif
		
		return;
	}
	
	/* Inflight ACC calibration */
	
	
	/* Multiple profiles change */
	
	
	/* Save configuration and notification */
	
	
	if (isUsingSticksToArm) {
		if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) {
			/* Arming via yaw stick */
			mwArm();
			return;
		}
	}
	
	/* Calibrate ACC process: Throttle TOP + Yaw to the LEFT + Pitch BOTTOM + Roll CENTRE
	 * 
	 * THR_HI = 2 << 6	 	(10000000)
	 * YAW_LO = 1 << 4		(00010000)
	 * PIT_LO = 1 << 2		(00000100)
	 * ROL_CE = 3 << 0		(00000011)
	 *
	 * THR_HI + YAW_LO + PIT_LO + ROL_CE = (2 << 6 | 1 << 4 | 1 << 2 | 3 << 0)
	 *									 = (10010111)
	 *									 = 151
	 */
	if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) {
//		printf("ACC Cali: %u, %s, %d\r\n", rcSticks, __FUNCTION__, __LINE__);
//		accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
		return;
	}
	
	/* Calibrate MAG process: Throttle TOP + Yaw to the RIGHT + Pitch BOTTOM + Roll CENTRE
	 * 
	 * THR_HI = 2 << 6	 	(10000000)
	 * YAW_HI = 2 << 4		(00100000)
	 * PIT_LO = 1 << 2		(00000100)
	 * ROL_CE = 3 << 0		(00000011)
	 *
	 * THR_HI + YAW_HI + PIT_LO + ROL_CE = (2 << 6 | 2 << 4 | 1 << 2 | 3 << 0)
	 *									 = (10100111)
	 *									 = 167
	 */
	if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) {
//		printf("MAG Cali: %u, %s, %d\r\n", rcSticks, __FUNCTION__, __LINE__);
//		ENABLE_STATE(CALIBRATE_MAG);
		return;
	}
	
	/* Change accelerometer trims */
	
	
	/* DASHBOARD */
	
	
	/* VTX */
	
	
	return;
}

//throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig)
{
	/* rxConfig->mincheck = 1100 */
	if (rcData[THROTTLE] < rxConfig->mincheck)
		return THROTTLE_LOW;
	
	return THROTTLE_HIGH;
}
