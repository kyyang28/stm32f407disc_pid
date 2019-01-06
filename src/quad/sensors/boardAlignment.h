#ifndef __BOARDALIGNMENT_H
#define __BOARDALIGNMENT_H

#include <stdint.h>

typedef struct boardAlignment_s {
	int32_t rollDegrees;
	int32_t pitchDegrees;
	int32_t yawDegrees;
}boardAlignment_t;

void initBoardAlignment(const boardAlignment_t *boardAlignment);
void alignSensors(int32_t *dest, uint8_t rotation);

#endif	// __BOARDALIGNMENT_H
