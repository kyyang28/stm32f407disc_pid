
#include <stdio.h>

#include <stdbool.h>
#include "maths.h"
#include "boardAlignment.h"
#include "axis.h"

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
	
//	printf("isStandardBoardAlignment: %d\r\n", isStandardBoardAlignment);
	fp_angles_t rotationAngles;
	rotationAngles.angles.roll = degreesToRadians(boardAlignment->rollDegrees);
	rotationAngles.angles.pitch = degreesToRadians(boardAlignment->pitchDegrees);
	rotationAngles.angles.yaw = degreesToRadians(boardAlignment->yawDegrees);
	
	buildRotationMatrix(&rotationAngles, boardRotation);

	// yaw = 1.5707963267948966192313216916398
//	for (int i = 0; i < 3; i++) {
//		for (int j = 0; j < 3; j++) {
//			printf("%f ", boardRotation[i][j]);
//			if (j == 2) printf("\r\n");
//		}
//	}
}

//static void alignBoard(int32_t *vec)
void alignBoard(int32_t *vec)
{
	int32_t x = vec[X];
	int32_t y = vec[Y];
	int32_t z = vec[Z];

//	for (int i = 0; i < 3; i++) {
//		for (int j = 0; j < 3; j++) {
//			printf("%f ", boardRotation[i][j]);
//			if (j == 2) printf("\r\n");
//		}
//	}
	
	vec[X] = lrintf(boardRotation[0][X] * x + boardRotation[1][X] * y + boardRotation[2][X] * z);
	vec[Y] = lrintf(boardRotation[0][Y] * x + boardRotation[1][Y] * y + boardRotation[2][Y] * z);
	vec[Z] = lrintf(boardRotation[0][Z] * x + boardRotation[1][Z] * y + boardRotation[2][Z] * z);
}

/* dest = gyroADC or acc.accSmooth or mag.magADC */
void alignSensors(int32_t *dest, uint8_t rotation)
{
	if (!isStandardBoardAlignment) {
//		alignBoard(dest);
	}
}
