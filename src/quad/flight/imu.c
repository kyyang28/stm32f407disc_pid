
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

static void imuComputeRotationMatrix(void)
{
	
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

static void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
	
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
