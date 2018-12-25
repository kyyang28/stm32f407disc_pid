
#include <stdio.h>

#include "rc_controls.h"
#include "fc_core.h"
#include "runtime_config.h"
#include "rx.h"				// rcData[]
#include "maths.h"			// constrain

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
//	printf("rcData: %u\r\n", rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT]);
	
//	uint16_t channelValue = constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
//	
//	return (channelValue >= 900 + (range->startStep * 25) && channelValue < 900 + (range->endStep * 25));
}

/** Update activated modes (BOXARM, BOXAIRMODE and so on)
 *
 */
void updateActivatedModes(modeActivationCondition_t *modeActivationConditions)
{
	rcModeActivationMask = 0;
	
	for (uint8_t index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
		modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];

//		printf("auxChannelIndex: %u\r\n", modeActivationCondition->auxChannelIndex);
//		printf("modeId: %d\r\n", modeActivationCondition->modeId);
//		printf("startStep: %u\r\n", modeActivationCondition->range.startStep);
//		printf("endStep: %u\r\n", modeActivationCondition->range.endStep);
		if (isRangeActive(modeActivationCondition->auxChannelIndex, &modeActivationCondition->range)) {
//			ACTIVATE_RC_MODE(modeActivationCondition->modeId);
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
