
#include <stdio.h>          // just for debugging purposes

#include "pid.h"
#include "fc_rc.h"
#include "mixer.h"
#include "configMaster.h"   // just for testing purposes

//#define TESTING_TPA

#if defined(TESTING_TPA)
#include "rx.h"
extern int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
#endif

/* 2-DOF PID controller based on MATLAB */
void pidController(const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim)
{
    static float previousRateError[2];
    const float tpaFactor = getThrottlePIDAttenuation();
    const float motorMixRange = getMotorMixRange();

#if defined(TESTING_TPA)    
    /* Throttle PID Attenuation (in %): 88%            Throttle value: 1617            TPA threshold: 1350 */
    printf("Throttle PID Attenuation (in %%): %d%%\tThrottle value: %d\tTPA threshold: %u\r\n", (int)(tpaFactor*100), rcData[THROTTLE], currentProfile->controlRateProfile[0].tpa_breakpoint);
#endif    
    
    
}
