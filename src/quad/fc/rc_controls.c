
#include <stdio.h>

#include "rc_controls.h"
#include "fc_core.h"
#include "runtime_config.h"
#include "rx.h"				// rcData[]
#include "maths.h"			// constrain
#include "sound_beeper.h"

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
			if (IS_RC_MODE_ACTIVE(BOXARM)) {
				printf("Motors are ARMed!\r\n");
			}
						
			if (IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
				printf("Buzzer is ON!\r\n");
				BEEP_ON;
			}
			
			if (!IS_RC_MODE_ACTIVE(BOXBEEPERON)) {
				BEEP_OFF;
			}
			
			if (IS_RC_MODE_ACTIVE(BOXANGLE)) {
				printf("Angle mode is ON!\r\n");
			}

			if (IS_RC_MODE_ACTIVE(BOXHORIZON)) {
				printf("HORIZON mode is ON!\r\n");
			}
			
//			if (!IS_RC_MODE_ACTIVE(BOXANGLE) && !IS_RC_MODE_ACTIVE(BOXHORIZON)) {
//				printf("MANUAL mode is ON!\r\n");
//			}
			
			if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
				printf("AIRMODE is ON!\r\n");
			}
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

void processRcStickPositions(void)
{
//	printf("armingFlags: %u, %s, %d\r\n", armingFlags, __FUNCTION__, __LINE__);
	
	if (CHECK_ARMING_FLAG(OK_TO_ARM)) {
//		printf("OK_TO_ARM: %s, %d\r\n", __FUNCTION__, __LINE__);
		mwArm();
	}
}
