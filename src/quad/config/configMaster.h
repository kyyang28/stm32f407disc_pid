#ifndef __CONFIGMASTER_H
#define __CONFIGMASTER_H

#include "led.h"
#include "ledTimer.h"
#include "sound_beeper.h"
#include "serial.h"
#include "gyro.h"
#include "acceleration.h"
#include "boardAlignment.h"
#include "rx_pwm.h"
#include "rx.h"
#include "rc_controls.h"
#include "platform.h"			// including target.h
#include "motors.h"				// including mixer.h
#include "config_profile.h"
#include "sdcard.h"
#include "blackbox.h"
//#include "pid.h"
#include "imu.h"				// including pid.h

typedef struct master_s {
	uint8_t version;
	uint16_t size;
	uint8_t magic_be;			// magic number, should be 0xBE
	
	uint32_t enabledFeatures;
	
	uint8_t debug_mode;
	uint8_t task_statistics;
	
	/* RADIO rx related configuration */
	rxConfig_t rxConfig;
	
	/* SERIAL related configuration */
	serialPinConfig_t serialPinConfig;
	serialConfig_t	serialConfig;
	
	/* IMU related configuration */
	gyroConfig_t gyroConfig;
	accelerometerConfig_t accelerometerConfig;
	imuConfig_t imuConfig;
	
	/* Board alignment configuration */
	boardAlignment_t boardAlignment;
	
    /* PID related configuration */
	pidConfig_t pidConfig;

	/* Remote control related configuration */
	rcControlsConfig_t rcControlsConfig;
	
	/* Throttle related configuration */
	throttleCorrectionConfig_t throttleCorrectionConfig;
	
	armingConfig_t armingConfig;
	
	/* LED related configuration */
	LedStatusConfig_t ledStatusConfig;
	LedTimerConfig_t ledTimerConfig;
	
	/* MOTOR related configuration */
	motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];
	motorConfig_t motorConfig;
	
	/* Mixer related configuration */
	mixerConfig_t mixerConfig;
	
	/* RADIO PWM INPUT related configuration */
#ifdef USE_PWM
	pwmConfig_t pwmConfig;
#endif
	
	/* BEEPER related configuration */
#ifdef BEEPER
	beeperConfig_t beeperConfig;
#endif
	uint32_t beeper_off_flags;
	uint32_t preferred_beeper_off_flags;	
	
#ifdef USE_SDCARD
	sdcardConfig_t sdcardConfig;
#endif

#ifdef BLACKBOX
	blackboxConfig_t blackboxConfig;
#endif
	
	profile_t profile[MAX_PROFILE_COUNT];
	uint8_t current_profile_index;
	
	modeActivationProfile_t modeActivationProfile;
	
	char boardIdentifier[sizeof(TARGET_BOARD_IDENTIFIER)];
	
	uint8_t magic_ef;							// magic number, should be 0xEF
	uint8_t chk;								// XOR checksum
	/*
	 * do not add properties after the MAGIC_EF and CHK
	 * as it is assumed to exist at length-2 and length-1
	 */
}master_t;

extern master_t masterConfig;
extern profile_t *currentProfile;
extern controlRateConfig_t *currentControlRateProfile;

#define LedStatusConfig(x)					(&masterConfig.ledStatusConfig)
#define LedTimerConfig(x)					(&masterConfig.ledTimerConfig)
#define BeeperConfig(x)						(&masterConfig.beeperConfig)
#define SerialPinConfig(x) 					(&masterConfig.serialPinConfig)
#define SerialConfig(x)						(&masterConfig.serialConfig)
#define MotorConfig(x)						(&masterConfig.motorConfig)
#define MixerConfig(x)						(&masterConfig.mixerConfig)
#define PwmConfig(x)						(&masterConfig.pwmConfig)
#define GyroConfig(x)						(&masterConfig.gyroConfig)
#define AccelerometerConfig(x)				(&masterConfig.accelerometerConfig)
#define ImuConfig(x)						(&masterConfig.imuConfig)
#define RxConfig(x)							(&masterConfig.rxConfig)
#define RcControlsConfig(x)					(&masterConfig.rcControlsConfig)
#define ArmingConfig(x)						(&masterConfig.armingConfig)
#define ModeActivationProfile(x)			(&masterConfig.modeActivationProfile)
#define SdcardConfig(x)						(&masterConfig.sdcardConfig)
#define BlackboxConfig(x)					(&masterConfig.blackboxConfig)
#define PidConfig(x)                        (&masterConfig.pidConfig)
#define BoardAlignment(x)					(&masterConfig.boardAlignment)
#define ThrottleCorrectionConfig(x)			(&masterConfig.throttleCorrectionConfig)

#endif	// __CONFIGMASTER_H
