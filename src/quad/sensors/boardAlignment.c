
#include <stdio.h>

#include <stdbool.h>
#include "boardAlignment.h"

static bool isStandardBoardAlignment = true;			// board orientation correction
static float boardRotation[3][3];					// matrix

/* If rollDegrees = pitchDegrees = yawDegrees = 0, boardAlignment is standard */
static bool isBoardAlignmentStandard(const boardAlignment_t *boardAlignment)
{
	return !boardAlignment->rollDegrees && !boardAlignment->pitchDegrees && !boardAlignment->yawDegrees;
}

void initBoardAlignment(const boardAlignment_t *boardAlignment)
{
//	printf("rollDeg: %d\r\n", boardAlignment->rollDegrees);
//	printf("pitchDeg: %d\r\n", boardAlignment->pitchDegrees);
//	printf("yawDeg: %d\r\n", boardAlignment->yawDegrees);
	
	if (isBoardAlignmentStandard(boardAlignment)) {
		return;
	}
	
	isStandardBoardAlignment = false;
	
	printf("isStandardBoardAlignment: %d\r\n", isStandardBoardAlignment);
}
