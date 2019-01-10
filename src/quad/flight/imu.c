
#include <stdio.h>

#include "imu.h"
#include "runtime_config.h"
#include "sensors.h"
#include "acceleration.h"
#include "maths.h"

float smallAngleCosZ = 0;
float accVelScale;
float fc_acc;
float throttleAngleScale;

static imuRuntimeConfig_t imuRuntimeConfig;
static pidProfile_t *pidProfile;

/* quaternion of sensor frame relative to earth frame */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float rMat[3][3];

static void imuComputeRotationMatrix(void)
{
	float q1q1 = sq(q1);
	float q2q2 = sq(q2);
	float q3q3 = sq(q3);
	
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q3 = q2 * q3;
	
	rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
	rMat[0][1] = 2.0f * (q1q2 + -q0q3);
	rMat[0][2] = 2.0f * (q1q3 - -q0q2);
	
	rMat[1][0] = 2.0f * (q1q2 - -q0q3);
	rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
	rMat[1][2] = 2.0f * (q2q3 + -q0q1);
	
	rMat[2][0] = 2.0f * (q1q3 + -q0q2);
	rMat[2][1] = 2.0f * (q2q3 - -q0q1);
	rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuInit(void)
{
	/* smallAngle for determining whether the quad is able to ARM or not */
	smallAngleCosZ = cosApprox(degreesToRadians(imuRuntimeConfig.smallAngle));
	accVelScale = 9.80665f / acc.dev.acc_1G / 10000.0f;
	
	imuComputeRotationMatrix();
}

/* Calculate RC time constant used in the accZ lpf */
float calculateAccZLowPassFilterRCTimeConstant(float accz_lpf_cutoff)
{
	/* accz_lpf_cutoff = 5.0f
	 *
	 * 0.5f / PI * 5.0f = 0.0318309886183790671537767526745
	 */
	return 0.5f / (M_PIf * accz_lpf_cutoff);
}

float calculateThrottleAngleScale(uint16_t throttleCorrectionAngle)
{
	return (1800.0f / M_PIf) * (900.0f / throttleCorrectionAngle);		// throttleCorrectionAnglem = 800, result: 644.57751952217610986397924165868
}

void imuConfigure(imuConfig_t *imuConfig, pidProfile_t *initialPidProfile, uint16_t throttleCorrectionAngle)
{
//	printf("dcm_kp: %u\r\n", imuConfig->dcm_kp);
//	printf("dcm_ki: %u\r\n", imuConfig->dcm_ki);
//	printf("accUnarmedcal: %u\r\n", imuConfig->accUnarmedcal);
//	printf("smallAngle: %u\r\n", imuConfig->smallAngle);
//	printf("throttleCorrectionAngle: %u\r\n", throttleCorrectionAngle);
	
	imuRuntimeConfig.dcm_kp = imuConfig->dcm_kp / 10000.0f;
	imuRuntimeConfig.dcm_ki = imuConfig->dcm_ki / 10000.0f;
	imuRuntimeConfig.accUnarmedcal = imuConfig->accUnarmedcal;
	imuRuntimeConfig.smallAngle = imuConfig->smallAngle;
		
	pidProfile = initialPidProfile;
	
//	printf("P8[0]: %u\r\n", pidProfile->P8[0]);
//	printf("P8[1]: %u\r\n", pidProfile->P8[1]);
//	printf("P8[2]: %u\r\n", pidProfile->P8[2]);

	/* fc_acc = 0.5f / PI * 5.0f = 0.0318309886183790671537767526745 */
	fc_acc = calculateAccZLowPassFilterRCTimeConstant(5.0f);	// fixed value 5.0f
	
	/* calculate throttle angle scale */
	throttleAngleScale = calculateThrottleAngleScale(throttleCorrectionAngle);
}

static bool imuIsAccelerometerHealthy(void)
{
	int32_t accMagnitude = 0;
	
	for (int32_t axis = 0; axis < 3; axis++) {
		accMagnitude += (int32_t)acc.accSmooth[axis] * acc.accSmooth[axis];
	}

//	printf("accMagnitude: %d\r\n", accMagnitude);
//	printf("sq((int32_t)acc.dev.acc_1G): %d\r\n", sq((int32_t)acc.dev.acc_1G));
	
	/* acc.dev.acc_1G = 512 * 8 = 4096 for MPU9250 configuration */
//	accMagnitude = sqrt(accMagnitude) * 100 / (int32_t)acc.dev.acc_1G;
	accMagnitude = accMagnitude * 100 / (sq((int32_t)acc.dev.acc_1G));

//	printf("accMagnitude: %d\r\n", accMagnitude);		// accMagnitude is between 102 and 103 when quad is not moving
	
	/* ACC readings should be within 0.90g - 1.10g */
	return (81 < accMagnitude) && (accMagnitude < 121);
}

//static bool isMagnetometerHealthy(void)
//{
//	return (mag.magADC[X] != 0) && (mag.magADC[Y] != 0) && mag.magADC[Z] != 0;
//}

static void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
	static uint32_t previousIMUUpdateTime;
	bool useAcc = false;
	bool useMag = false;
	bool useYaw = false;
	
	uint32_t deltaT = currentTimeUs - previousIMUUpdateTime;	// deltaT ~= 10000 us = 10 ms
	previousIMUUpdateTime = currentTimeUs;
	
//	printf("deltaT: %u\r\n", deltaT);			// deltaT ~= 10000 us = 10 ms
	
	if (imuIsAccelerometerHealthy()) {
		useAcc = true;
	}

	/* TODO: Implement later */
//	if (sensors(SENSOR_MAG) && isMagnetometerHealthy()) {
//		useMag = true;
//	}
//#if defined(GPS)
//	else if () {
//		
//	}
//#endif
	
	/* Perform MahonyAHRS algorithm */
//	imuMahonyAHRSUpdate();
	
	/* Update Euler angles */
//	imuUpdateEulerAngles();
	
	/* Calculate acceleration */
//	imuCalculateAcceleration(deltaT);
}

void taskIMUUpdateAttitude(timeUs_t currentTimeUs)
{
	if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		imuCalculateEstimatedAttitude(currentTimeUs);
	} else {
		acc.accSmooth[X] = 0;
		acc.accSmooth[Y] = 0;
		acc.accSmooth[Z] = 0;
	}
}
