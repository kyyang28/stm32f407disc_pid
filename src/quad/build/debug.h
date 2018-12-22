#ifndef __DEBUG_H
#define __DEBUG_H

#include <stdint.h>

#define DEBUG16_VALUE_COUNT			4

extern int16_t debug[DEBUG16_VALUE_COUNT];
extern uint8_t debugMode;

#define DEBUG_SET(mode, index, value)		{if (debugMode == (mode)) { debug[(index)] = (value); }}

typedef enum {
	DEBUG_NONE,
	DEBUG_CYCLETIME,
	DEBUG_BATTERY,
	DEBUG_GYRO,
	DEBUG_ACCELEROMETER,
	DEBUG_MIXER,
	DEBUG_AIRMODE,
	DEBUG_PIDLOOP,
	DEBUG_NOTCH,
	DEBUG_RC_INTERPOLATION,
	DEBUG_VELOCITY,
	DEBUG_DTERM_FILTER,
	DEBUG_ANGLERATE,
	DEBUG_ESC_SENSOR,
	DEBUG_SCHEDULER,
	DEBUG_STACK,
	DEBUG_COUNT
}debugType_e;

#endif	// __DEBUG_H
