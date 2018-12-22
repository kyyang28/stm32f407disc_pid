#ifndef __MOTORS_H
#define __MOTORS_H

#include "io.h"
#include "mixer.h"

typedef struct motorConfig_s {
    uint16_t minthrottle;		// Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;       // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;        // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t motorPwmRate;      // The update rate of motor outputs (50-498Hz)
    uint8_t  motorPwmProtocol;  // Pwm Protocol
    uint8_t  useUnsyncedPwm;
    float    digitalIdleOffsetPercent;
    ioTag_t  ioTags[MAX_SUPPORTED_MOTORS];
}motorConfig_t;

#endif	// __MOTORS_H
