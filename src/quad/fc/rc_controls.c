
#include <stdio.h>

#include "rc_controls.h"
#include "fc_core.h"
#include "runtime_config.h"

static motorConfig_t *motorConfig;
static pidProfile_t *pidProfile;

int16_t rcCommand[4];			// interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

/* true if arming is done via the sticks (as opposed to a switch) */
static bool isUsingSticksToArm = true;

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
