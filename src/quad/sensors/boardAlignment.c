
#include <stdio.h>

#include <stdbool.h>
#include "maths.h"
#include "boardAlignment.h"
#include "axis.h"
#include "sensor.h"

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

static void alignBoard(int32_t *vec)
{
	int32_t x = vec[X];
	int32_t y = vec[Y];
	int32_t z = vec[Z];

//	for (int i = 0; i < 3; i++) {
//		for (int j = 0; j < 3; j++) {
//			printf("(%d,%d): %f ", i, j, boardRotation[i][j]);
//			if (j == 2) printf("\r\n");
//		}
//	}
	
	/*
	                +-																		    -+ +-   -+
			        |	cosycosz		cosxsinz + coszsinysinx			sinxsinz - cosxcoszsiny  | |  x  |
	(RzRyRx)vec  =  |  -cosysinz		cosxcosz - sinxsinysinz			coszsinx + cosxsinysinz  | |  y  |
			        |	  siny				   -cosysinx					   cosycosx          | |  z  |
	                +-																		    -+ +-   -+
	 */
#if defined(RzRyRx)
	/* Rotation matrix initialisation */
//	matrix[0][0] = cosz * cosy;
//	matrix[0][1] = (sinzcosx) + (coszsinx * siny);
//	matrix[0][2] = (sinzsinx) - (coszcosx * siny);
//	matrix[1][0] = -cosy * sinz;
//	matrix[1][1] = (coszcosx) - (sinzsinx * siny);
//	matrix[1][2] = (coszsinx) + (sinzcosx * siny);
//	matrix[2][0] = siny;
//	matrix[2][1] = -sinx * cosy;
//	matrix[2][2] = cosy * cosx;
	vec[X] = lrintf(boardRotation[0][X] * x + boardRotation[0][Y] * y + boardRotation[0][Z] * z);
	vec[Y] = lrintf(boardRotation[1][X] * x + boardRotation[1][Y] * y + boardRotation[1][Z] * z);
	vec[Z] = lrintf(boardRotation[2][X] * x + boardRotation[2][Y] * y + boardRotation[2][Z] * z);
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
#else
	/* Rotation matrix initialisation */
//	matrix[0][X] = cosz * cosy;
//	matrix[0][Y] = -cosy * sinz;
//	matrix[0][Z] = siny;
//	matrix[1][X] = (sinzcosx) + (coszsinx * siny);
//	matrix[1][Y] = (coszcosx) - (sinzsinx * siny);
//	matrix[1][Z] = -sinx * cosy;
//	matrix[2][X] = (sinzsinx) - (coszcosx * siny);
//	matrix[2][Y] = (coszsinx) + (sinzcosx * siny);
//	matrix[2][Z] = cosy * cosx;
	vec[X] = lrintf(boardRotation[0][X] * x + boardRotation[1][X] * y + boardRotation[2][X] * z);
	vec[Y] = lrintf(boardRotation[0][Y] * x + boardRotation[1][Y] * y + boardRotation[2][Y] * z);
	vec[Z] = lrintf(boardRotation[0][Z] * x + boardRotation[1][Z] * y + boardRotation[2][Z] * z);
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
#endif
	
//	printf("vec[%d]: %d\r\n", X, vec[X]);
//	printf("vec[%d]: %d\r\n", Y, vec[Y]);
//	printf("vec[%d]: %d\r\n", Z, vec[Z]);
}

/* dest = gyroADC or acc.accSmooth or mag.magADC */
void alignSensors(int32_t *dest, uint8_t rotation)
{
	const int32_t x = dest[X];
	const int32_t y = dest[Y];
	const int32_t z = dest[Z];
	
//	printf("x: %i\ty: %i\tz: %i\r\n", x, y, z);
	
	switch (rotation) {
		default:
		case CW0_DEG:
			dest[X] = x;
			dest[Y] = y;
			dest[Z] = z;
			break;
		
		case CW90_DEG:
			dest[X] = y;
			dest[Y] = -x;
			dest[Z] = z;
			break;
		
		case CW180_DEG:
			dest[X] = -x;
			dest[Y] = -y;
			dest[Z] = z;
			break;
		
		case CW270_DEG:
			dest[X] = -y;
			dest[Y] = x;
			dest[Z] = z;
			break;
		
		case CW0_DEG_FLIP:
			dest[X] = -x;
			dest[Y] = y;
			dest[Z] = -z;
			break;
		
		case CW90_DEG_FLIP:
			dest[X] = y;
			dest[Y] = x;
			dest[Z] = -z;
			break;
		
		case CW180_DEG_FLIP:
			dest[X] = x;
			dest[Y] = -y;
			dest[Z] = -z;
			break;
		
		case CW270_DEG_FLIP:
			dest[X] = -y;
			dest[Y] = -x;
			dest[Z] = -z;
			break;
	}
	
	if (!isStandardBoardAlignment) {
		alignBoard(dest);
	}
}
