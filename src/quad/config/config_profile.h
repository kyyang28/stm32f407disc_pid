#ifndef __CONFIG_PROFILE_H
#define __CONFIG_PROFILE_H

#include "config.h"
#include "rc_controls.h"		// including pid.h

typedef struct profile_s {
	pidProfile_t pidProfile;
	uint8_t activeRateProfile;
	controlRateConfig_t controlRateProfile[MAX_RATEPROFILES];
}profile_t;

#endif	// __CONFIG_PROFILE_H
