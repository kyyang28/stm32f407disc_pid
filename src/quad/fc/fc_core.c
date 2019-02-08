
#include <stdio.h>			// testing purposes
#include "platform.h"
//#include "rx.h"				// including time.h
#include "debug.h"
#include "system.h"
#include "pwm_output.h"
#include "asyncfatfs.h"
#include "blackbox.h"
#include "runtime_config.h"
#include "led.h"
#include "gyro.h"
#include "configMaster.h"
//#include "pid.h"

/*  */
#include "rc_controls.h"	// including rx.h and time.h; print data (int16_t rcCommand[0-3])
#include "fc_rc.h"			// print data (float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3])
#include "mixer.h"			// including pid.h, print data (int16_t motor[MAX_SUPPORTED_MOTORS])

uint8_t motorControlEnable = false;

bool isRXDataNew;

static bool armingCalibrationWasInitialised;
static uint32_t disarmAt;		// Time of automatic disarm when "Don't spin the motors when armed" is enabled and auto_disarm_delay is nonzero.

void updateLEDs(void)
{
//	printf("armingFlags: %u, %s, %s, %d\r\n", armingFlags, __FILE__, __FUNCTION__, __LINE__);
	
//	printf("BOXARM: %u\r\n", IS_RC_MODE_ACTIVE(BOXARM));
	
	if (CHECK_ARMING_FLAG(ARMED)) {
//		printf("ARMED, %s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
//		LED3_ON; LED4_ON; LED5_ON; LED6_ON;
		LED6_ON;
	} else if (IS_RC_MODE_ACTIVE(BOXARM) == 0 || armingCalibrationWasInitialised) {
//		printf("Not ARMED: %s, %d\r\n", __FUNCTION__, __LINE__);
		ENABLE_ARMING_FLAG(OK_TO_ARM);
		LED6_OFF;
	}
}

void mwArm(void)
{
	static bool firstArmingCalibrationWasCompleted;
	
	/* ArmingConfig()->gyro_cal_on_first_arm = 0 */
	if (ArmingConfig()->gyro_cal_on_first_arm && !firstArmingCalibrationWasCompleted) {
		gyroSetCalibrationCycles();		// set gyroCalibrationCycle to 8000 if gyro.targetLooptime = 125 us
		armingCalibrationWasInitialised = true;
		firstArmingCalibrationWasCompleted = true;
	}
	
	/* Check gyro calibration before arming
	 * 
	 * Prevent arming before gyro is calibrated.
	 */
	if (!isGyroCalibrationComplete())
		return;
	
	if (CHECK_ARMING_FLAG(OK_TO_ARM)) {
		if (CHECK_ARMING_FLAG(ARMED)) {
			return;
		}
		
        if (IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
            return;
        }
		
		if (!CHECK_ARMING_FLAG(PREVENT_ARMING)) {
			ENABLE_ARMING_FLAG(ARMED);
			ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
//			headFreeModeHold = DECIDEGREES_TO_DEGREES(attitude.values.yaw);
			
#ifdef BLACKBOX
			if (feature(FEATURE_BLACKBOX)) {
				startBlackbox();
			}
#endif
			
			/* ArmingConfig()->auto_disarm_delay = 5 seconds
			 *
			 * disarmAt = current time in miliseconds + disarm_delay time in miliseconds
			 *			= millis() + 5 * 1000
			 * 			= millis() + 5000 (ms)
			 */
			disarmAt = millis() + ArmingConfig()->auto_disarm_delay * 1000;
			
			/* Beep to indicate arming status */
			// call beeper(BEEPER_ARMING)		// implement later
			
			return;
		}
	}

	/* Implement beeperConfirmationBeeps(1) later */
//	if (!CHECK_ARMING_FLAG(ARMED)) {
//		beeperConfirmationBeeps(1);
//	}
}

void mwDisarm(void)
{
	armingCalibrationWasInitialised = false;
	
	if (CHECK_ARMING_FLAG(ARMED)) {
		DISABLE_ARMING_FLAG(ARMED);
		
#ifdef BLACKBOX
		if (feature(FEATURE_BLACKBOX)) {
			finishBlackbox();
		}
#endif
	
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//		beeper(BEEPER_DISARMING);	TODO: implement later
	}
}

void processRx(timeUs_t currentTimeUs)
{
	static bool airmodeIsActivated;
	static bool armedBeeperOn = false;
	
	calculateRxChannelsAndUpdateFailsafe(currentTimeUs);
	
	/* update RSSI, IMPLEMENTATION LATER */
	
	
	/* handle failsafe if necessary, IMPLEMENTATION LATER */
	
	
	/* calculate throttle status */
	throttleStatus_e throttleStatus = calculateThrottleStatus(&masterConfig.rxConfig);
	
//	printf("AirMode: %d\r\n", isAirModeActive());		// 1: airmode active, 0: airmode is not active
//	printf("ARMED: %d\r\n", CHECK_ARMING_FLAG(ARMED));
	
	/* +------------------------------ Handle AirMode ------------------------------+ */
	if (isAirModeActive() && CHECK_ARMING_FLAG(ARMED)) {
//		printf("rcCommand[THROTTLE]: %u\r\n", rcCommand[THROTTLE]);		// rcCommand[THROTTLE] ranges from 1000 to 2000 us
		
		/* airModeActivateThreshold = 1350 */
		if (rcCommand[THROTTLE] >= RxConfig()->airModeActivateThreshold) {
			
			/* airmode is activated IF AND ONLY IF the quad is ARMED and airmode switch has been TURNED ON */
			airmodeIsActivated = true;
		}
	} else {
		
		/* airmode is deactivated IF the quad is DISARMED or the airmode switch has been TURNED OFF */
		airmodeIsActivated = false;
	}
	
//	printf("airmodeIsActivated: %d\r\n", airmodeIsActivated);
	
	/*
	 * In AirMode, I-term should not be grown when low throttle and roll, pitch sticks are at the centre position
	 * This is needed to prevent I-term winding on the ground, but keep full stabilisation on 0 throttle while in air
	 */
	if (throttleStatus == THROTTLE_LOW && !airmodeIsActivated) {
		pidResetErrorGyroState();
		
		/* currentProfile->pidProfile.pidAtMinThrottle = PID_STABILISATION_ON = 1 */
		if (currentProfile->pidProfile.pidAtMinThrottle) {
			/* PID stablisation is on */
			pidStabilisationState(PID_STABILISATION_ON);
		} else {
			pidStabilisationState(PID_STABILISATION_OFF);
		}
	} else {
		/* throttleStatus != THROTTLE_LOW or airmodeIsActivated is TRUE
		 * Meaning throttle value is ABOVE mincheck (1100) or airmode is ON,
		 * 
		 * PID stabilisation should be switched on.
		 */
		pidStabilisationState(PID_STABILISATION_ON);
	}
	
//	printf("ARMED: %d\r\n", CHECK_ARMING_FLAG(ARMED));
//	printf("MOTOR STOP: %u\r\n", feature(FEATURE_MOTOR_STOP));
//	printf("AirMode: %d\r\n", !isAirModeActive());
	
	/* When ARMed and motors are not spinning, perform beeps and then disarm flight controller board after delay
	 * so users without using buzzer will not lose fingers
	 *
	 * mixTable() function contains motor commands, so checking throttleStatus is enough.
	 *
	 * ARMed && MOTOR is not spinning && AirMode is not activated.
	 *
	 * Normally, if AirMode is permanently ON, this if statement won't be executed even if it is ARMed and motors are not spinning.
	 */
	if (CHECK_ARMING_FLAG(ARMED) && feature(FEATURE_MOTOR_STOP) && !isAirModeActive()) {
		if (isUsingSticksForArming()) {
			/* TODO: implement later */
		} else {
			/* ARMing via AUX switch (ARM BOX), beep while throttle is low */
			if (throttleStatus == THROTTLE_LOW) {
//				beeper(BEEPER_ARMED);
				armedBeeperOn = true;
//				printf("armedBeeperOn: %d, %d\r\n", armedBeeperOn, __LINE__);
			} else if (armedBeeperOn) {
//				beeperSilence();
				armedBeeperOn = false;
//				printf("armedBeeperOn: %d, %d\r\n", armedBeeperOn, __LINE__);
			}
		}
	}
	
	/* Handle RC stick positions
	 *
	 * throttleStatus = THROTTLE_LOW (rcData[THROTTLE] < rxConfig->mincheck) or THROTTLE_HIGH (rcData[THROTTLE] >= rxConfig->mincheck)
	 * ArmingConfig()->disarm_kill_switch = 1
	 */
	processRcStickPositions(&masterConfig.rxConfig, throttleStatus, ArmingConfig()->disarm_kill_switch);
	
	/* update activated modes */
	updateActivatedModes(ModeActivationProfile()->modeActivationConditions);
	
	/* TODO: Implement later */
//	if (!cliMode) {
//		updateAdjustmentStates(adjustmentProfile()->adjustmentRanges);
//		processRcAdjustments(currentControlRateProfile, RxConfig());
//	}
	
	/* Flight mode initialisations */
	bool canUseHorizonMode = true;
	
	/* Angle mode */
	if ((IS_RC_MODE_ACTIVE(BOXANGLE) || (feature(FEATURE_FAILSAFE) /* && failsafeIsActive() */)) && (sensors(SENSOR_ACC))) {
		canUseHorizonMode = false;
		
		if (!FLIGHT_MODE(ANGLE_MODE)) {
			ENABLE_FLIGHT_MODE(ANGLE_MODE);
		}
	} else {
		DISABLE_FLIGHT_MODE(ANGLE_MODE);
	}
	
	/* Horizon mode */
	if (IS_RC_MODE_ACTIVE(BOXHORIZON) && canUseHorizonMode) {
		DISABLE_FLIGHT_MODE(ANGLE_MODE);
		
		if (!FLIGHT_MODE(HORIZON_MODE)) {
			ENABLE_FLIGHT_MODE(HORIZON_MODE);
		}
	} else {
		DISABLE_FLIGHT_MODE(HORIZON_MODE);
	}
	
	if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
		LED3_ON;
	} else {
		LED3_OFF;
	}
	
#if defined(ACC) || defined(MAG)
	if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
		/* TODO: Implement later */
	}
#endif
	
#ifdef GPS
	if (sensors(SENSOR_GPS)) {
		/* TODO: Implement later */
		updateGPSWaypointsAndMode();
	}
#endif
	
	/* TODO: BOXPASSTHRU mode and FIX WING, AIRPLANE handler should be implemented here */

#ifdef TELEMETRY
	/* TODO: Implement later */
#endif
	
#ifdef VTX
	/* TODO: Implement later */
#endif
}

static void subTaskMotorUpdate(void)
{
	uint32_t startTime;
	if (debugMode == DEBUG_CYCLETIME) {
		startTime = micros();
		static uint32_t previousMotorUpdateTime;		// static keyword to keep the previous motor update time
		const uint32_t currentDeltaTime = startTime - previousMotorUpdateTime;
		debug[2] = currentDeltaTime;
//		debug[3] = currentDeltaTime - targetPidLooptime;		// TODO: targetPidLooptime is defined in pid.c
		previousMotorUpdateTime = startTime;
	}else if (debugMode == DEBUG_PIDLOOP) {
		startTime = micros();
	}
	
    /**
     * mixTable(&currentProfile->pidProfile) function performs the following tasks.
     *
     * 1. Find min and max throttle value based on condition (non-3D mode or 3D mode)
     * 2. Constrain throttle value
     * 3. Calculate and limit PID sum
     * 4. Calculate voltage compensation
     * 5. Find Roll/Pitch/Yaw desired outputs
     * 6. Adjust throttle value during airmode condition
     * 7. Update motor[i] values for writeMotors() function to control four motors
     */
	mixTable(&currentProfile->pidProfile);
	
	if (motorControlEnable) {
//		printf("motorControlEnable: %s, %d\r\n", __FUNCTION__, __LINE__);
        
        /*
         * Update each motor[i] value. ( pwmWriteMotor(i, motor[i]) )
         */
        writeMotors();
	}
}

static void subTaskMainSubprocesses(timeUs_t currentTimeUs)
{
	/* Calculate throttle value for ANGLE or HORIZON modes */
	
	
	/* Process RC commands */
	processRcCommand();
	
	/* Polling SDcard data */
#ifdef USE_SDCARD
	afatfs_poll();
#endif
	
	/* Store data to blackbox (SDcard) */
#ifdef BLACKBOX
	if (/*!cliMode && */feature(FEATURE_BLACKBOX)) {
		handleBlackbox(currentTimeUs);
	}
#endif
}

uint8_t getUpdatedPIDCountDown(void)
{
    if (GyroConfig()->gyro_soft_lpf_hz) {
        return PidConfig()->pid_process_denom - 1;
    } else {
        return 1;
    }
    
    return 0;
}

static void subTaskPidController(void)
{
    pidController(&currentProfile->pidProfile, &AccelerometerConfig()->accelerometerTrims);
}

void taskMainPidLoop(timeUs_t currentTimeUs)
{
	static bool runTaskMainSubprocesses;
    static uint8_t pidUpdateProcessCountDown;
	
	/* run subTaskMainSubprocesses */
	if (runTaskMainSubprocesses) {
		subTaskMainSubprocesses(currentTimeUs);
		runTaskMainSubprocesses = false;
	}
	
    /* DEBUG_PIDLOOP, timings for:
     * 0 - gyroUpdate()
     * 1 - pidController()
     * 2 - subTaskMainSubprocesses()
     * 3 - subTaskMotorUpdate()
	 */
	
	/* gyroUpdate */
	gyroUpdate(currentTimeUs);
    
    if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_SPI && gyro.dev.calibrationFlag) {
//        printf("%u,%.4f,%.4f,%.4f\r\n", currentTimeUs, gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z]);
    }
    
//    printf("pidUpdateProcessCountDown: %u\r\n", pidUpdateProcessCountDown);
    
    if (pidUpdateProcessCountDown) {
        pidUpdateProcessCountDown--;
    } else {
        pidUpdateProcessCountDown = getUpdatedPIDCountDown();       // Example: pidUpdateProcessCountDown = PidConfig()->pid_process_denom - 1 = 4 - 1 = 3
        
        /* subTaskPidController */
        subTaskPidController();
        
        /* subTaskMotorUpdate
         *
         * How can I run the PID controller faster than 2kHz ?
         *
         * Set looptime (microSeconds) in config GUI. OneShot42 and MultiShot now supported
         *
         * 2 examples of auto config
         *
         * looptime 125
         *
         * always 8khz gyro sampling (gyro_sync_denom = 1)
         * when just oneshot125:
         * pid_process_denom = 3
         * when use_oneshot42 or use_multishot
         * pid_process_denom = 2
         * looptime 250
         *
         * always 4k gyro sampling (gyro_sync_denom = 2)
         * pid_process_denom = 2
         * on f1 boards with luxfloat
         * pid_process_denom = 3
         *
         * motor update speed = pid speed calculation of motor speed: motor update interval us = 125 * gyro_sync_denom * pid_process_denom
         * 
         * PID is always synced to motors! PID speed is immediately your motor update speed. Gyro can run faster than PID. 
         * The benefit of that is the higher sampling reduces filtering delays and helps catching up all higher frequencies that may fold down into lower 
         * frequencies when undersampled. Even when GYRO runs faster than PID it is still in sync, but every (pid_process_denom)th sample.
         */
        subTaskMotorUpdate();
        runTaskMainSubprocesses = true;
		
		/* Display data via SecureCRT (which logs the data at the same time) for MATLAB data analysis
		 *
		 * 0: ROLL, 1: PITCH, 2: YAW, 3: THROTTLE
		 *
		 * 1. int16_t rcCommand[0-3];			// rcCommandf[0-3] = rcCommand[0-3] / 500;
		 * 2. float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3];
		 * 3. int16_t motor[MAX_SUPPORTED_MOTORS];
		 */
		printf("%.4f,%d,%d,%d,%d,%.4f,%.4f,%.4f,%d,%d,%d,%d\r\n", currentTimeUs/1000000.0f, rcCommand[0], rcCommand[1], rcCommand[2], rcCommand[3], 
								setpointRate[0], setpointRate[1], setpointRate[2], motor[0], motor[1], motor[2], motor[3]);
    }
}
