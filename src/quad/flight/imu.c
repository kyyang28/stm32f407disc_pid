
#include <stdio.h>

#include "imu.h"
#include "runtime_config.h"
#include "sensors.h"
#include "acceleration.h"
#include "maths.h"
#include "system.h"
#include "gyro.h"
#include "acceleration.h"
#include "compass.h"

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://www.acsel-lab.com/arithmetic/arith13/papers/ARITH13_Hekstra.pdf (fast rotation)
#define GYRO_RATE_LIMIT							20

float smallAngleCosZ = 0;
float accVelScale;
float fc_acc;
float throttleAngleScale;

static imuRuntimeConfig_t imuRuntimeConfig;
static pidProfile_t *pidProfile;

/* quaternion of sensor frame relative to earth frame */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float rMat[3][3];

/* The absolute angle inclination is in multiple of 0.1 degree, which means 180 deg = 1800 */
attitudeEulerAngles_t attitude = { {0, 0, 0} };

/* magneticDeclination is calculated from config */
float magneticDeclination = 0.0f;

static float invSqrt(float x)
{
	return 1.0f / sqrtf(x);
}

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

static bool imuUseFastGains(void)
{
	return !CHECK_ARMING_FLAG(ARMED) && millis() < 20000;			// 20000 ms = 20 sec
}

static float imuGetPGainScaleFactor(void)
{
	if (imuUseFastGains()) {
		return 10.0f;
	} else {
		return 1.0f;
	}
}

static void imuMahonyAHRSUpdate(timeUs_t currentTimeUs, float dt, float gx, float gy, float gz, bool useAcc, float ax, float ay, float az,
							bool useMag, float mx, float my, float mz, bool useYaw, float yawError)
{
	/* Integral error terms scaled by Ki */
	static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
	float hx, hy, bx;
	float ex = 0, ey = 0, ez = 0;
	float qa, qb, qc;
	float recipNorm;
	
	/* Step 1: Calculate the gyro angular rate (rad/s) */
	float gyro_ang_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));
	
	/* Step 2: If useYaw is true, use raw heading error (from GPS sensor or something else) */
	if (useYaw) {
		/* TODO: Implementation for GPS data fusion later */
	}
	
	/* Step 3: If magnetic data is available, use measured magnetic field vector */
	recipNorm = sq(mx) + sq(my) + sq(mz);
	if (useMag && recipNorm > 0.01f) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		/* Normalise magnetometor data */
		recipNorm = invSqrt(recipNorm);
//		recipNorm = fastInvSqrt(recipNorm);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;
		
        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles

        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
        bx = sqrtf(hx * hx + hy * hy);

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        float ez_ef = -(hy * bx);

        // Rotate mag error vector back to BF and accumulate
        ex += rMat[2][0] * ez_ef;
        ey += rMat[2][1] * ez_ef;
        ez += rMat[2][2] * ez_ef;
	}
	
	/* Step 4: If accelerometer data is available, use measured accelerometer vector */
	recipNorm = sq(ax) + sq(ay) + sq(az);
	if (useAcc && recipNorm > 0.01f) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		/* Normalise accelerometer data */
		recipNorm = invSqrt(recipNorm);
//		recipNorm = fastInvSqrt(recipNorm);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		
//		printf("ax: %f, ay: %f, az: %f\r\n", ax, ay, az);		// Normalised acc data

		/* Accelerometer error is the sum of cross product between estimated vector and measured vector of gravity */
		ex += (ay * rMat[2][2] - az * rMat[2][1]);
		ey += (az * rMat[2][0] - ax * rMat[2][2]);
		ez += (ax * rMat[2][1] - ay * rMat[2][0]);
	}
	
	/*
	 * Step 5: Calculate the Integration Part (Ki).
	 */
	if (imuRuntimeConfig.dcm_ki > 0.0f) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		/* Stop integrating if gyro rate is beyond the certain limit */
		if (gyro_ang_rate < DEGREES_TO_RADIANS(GYRO_RATE_LIMIT)) {
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			float dcmKiGain = imuRuntimeConfig.dcm_ki;
			
			/* Integrate(Accumulate) the error terms (ex, ey, ez) */
			integralFBx += dcmKiGain * ex * dt;
			integralFBy += dcmKiGain * ey * dt;
			integralFBz += dcmKiGain * ez * dt;
		} else {
			/* If dcm_ki is zero, disable the integration */
//			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
	}
	
	/* Step 6: Calculate the Proportional Part (Kp).
	 *
	 * If the quad is powered up and not armed within 20 sec, boost the Kp parameter to converge the error term faster
	 *
	 * imuGetPGainScaleFactor() either returns 10.0f or 1.0f
	 */
	float dcmKpGain = imuRuntimeConfig.dcm_kp * imuGetPGainScaleFactor();
//	printf("imuRuntimeConfig.dcm_kp: %.4f\r\n", imuRuntimeConfig.dcm_kp);
//	printf("dcmKpGain: %.4f\r\n", dcmKpGain);
	
	/* Step 7: Apply PI controller to minimise the steady-state error e = [ex, ey, ez] */
//	printf("%.4f, %.4f, %.4f\r\n", dcmKpGain * ex + integralFBx, dcmKpGain * ex + integralFBx, dcmKpGain * ez + integralFBz);
	gx += dcmKpGain * ex + integralFBx;				// Compensate gyro drift on x-axis
	gy += dcmKpGain * ey + integralFBy;				// Compensate gyro drift on y-axis
	gz += dcmKpGain * ez + integralFBz;				// Compensate gyro drift on z-axis
	
//	printf("%u,%.4f,%.4f,%.4f\r\n", currentTimeUs, gx, gy, gz);		// Gyroscope data after applying drift compensator using accelerometer
	
	/* Step 8: Calculate the rate of change of quaternion (derivative of quaternion) */
	gx *= (0.5f * dt);
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	
	qa = q0;
	qb = q1;
	qc = q2;
	
	/* q0, q1, q2, q3 are calculated by taking the integration of the derivative of the quaternion */
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);
	
//	printf("q0:%.4f,q1:%.4f,q2:%.4f,q3:%.4f\r\n", q0, q1, q2, q3);
	
	/* Step 9: Normalise the rate of change of quaternion */
	recipNorm = invSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
//	recipNorm = fastInvSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

//	printf("q0:%.4f,q1:%.4f,q2:%.4f,q3:%.4f\r\n", q0, q1, q2, q3);
	
	/* Step 10: Compute the rotation matrix from the updated quaternion */
	imuComputeRotationMatrix();
}

static void imuUpdateEulerAngles(timeUs_t currentTimeUs)
{
	/* Compute Euler angles(Roll/Pitch/Yaw) based on Rotation Matrix */
//	attitude.values.roll = lrintf(atan2f(rMat[2][1], rMat[2][2]) * (180.0f / M_PIf));
//	attitude.values.pitch = lrintf(((0.5f * M_PIf) - acosf(-rMat[2][0])) * (180.0f / M_PIf));
//	attitude.values.yaw = lrintf(-atan2f(rMat[1][0], rMat[0][0]) * (180.0f / M_PIf) + magneticDeclination);
	attitude.values.roll = lrintf(atan2f(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
	attitude.values.pitch = lrintf(((0.5f * M_PIf) - acosf(-rMat[2][0])) * (1800.0f / M_PIf));
	attitude.values.yaw = lrintf(-atan2f(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf) + magneticDeclination);

//	printf("rawRoll: %.4f,Roll: %ld\r\n", atan2f(rMat[2][1], rMat[2][2]), lrintf(atan2f(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf)));
	
//	printf("%u,%d,%d,%d\r\n", currentTimeUs, attitude.values.roll, attitude.values.pitch, attitude.values.yaw);
	
	if (attitude.values.yaw < 0) {
		attitude.values.yaw += 3600;
//		attitude.values.yaw += 360;
	}

//	printf("roll: %d, pitch: %d, yaw: %d\r\n", attitude.values.roll, attitude.values.pitch, attitude.values.yaw);
	
//	printf("roll: %d, pitch: %d, yaw: %d\r\n", attitude.values.roll, attitude.values.pitch, attitude.values.yaw);
	
	/* Update small angle */
//	printf("rMat[2][2]: %f, smallAngleCosz: %f\r\n", rMat[2][2], smallAngleCosZ);
	if (rMat[2][2] > smallAngleCosZ) {
		ENABLE_STATE(SMALL_ANGLE);
	} else {
		DISABLE_STATE(SMALL_ANGLE);
	}
}

static void imuCalculateEstimatedAttitude(timeUs_t currentTimeUs)
{
	static uint32_t previousIMUUpdateTime;
	bool useAcc = false;
	bool useMag = false;
	bool useYaw = false;
	float rawYawError = 0;
	
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
	imuMahonyAHRSUpdate(currentTimeUs, deltaT * 1e-6f, DEGREES_TO_RADIANS(gyro.gyroADCf[X]), DEGREES_TO_RADIANS(gyro.gyroADCf[Y]), DEGREES_TO_RADIANS(gyro.gyroADCf[Z]),
						useAcc, acc.accSmooth[X], acc.accSmooth[Y], acc.accSmooth[Z], useMag, mag.magADC[X], mag.magADC[Y], mag.magADC[Z], useYaw, rawYawError);
	
	/* Update Euler angles */
	imuUpdateEulerAngles(currentTimeUs);
	
	/* Calculate acceleration for altitude (height) and position control */
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
