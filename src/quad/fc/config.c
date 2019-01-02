
#include <stdio.h>				// debugging purposes
#include <string.h>
#include "configMaster.h"		// including led.h, ledTimer.h, motors.h, sound_beeper.h, target.h, serial.h
#include "config_profile.h"
#include "config_eeprom.h"
//#include "led.h"
//#include "ledTimer.h"			// for testing led timer ONLY
//#include "motors.h"				// including mixer.h
//#include "sound_beeper.h"
//#include "target.h"
//#include "serial.h"
#include "sensor.h"
#include "pwm_output.h"			// including timer.h
#include "feature.h"
#include "accgyro.h"
#include "acceleration.h"
#include "filter.h"
#include "rc_controls.h"
#include "rx.h"
#include "common.h"
#include "maths.h"
#include "mixer.h"
#include "fc_rc.h"
#include "sdcard.h"
#include "blackbox_io.h"

#define BRUSHED_MOTORS_PWM_RATE 			16000
#define BRUSHLESS_MOTORS_PWM_RATE 			480

/* master config structure with data independent from profiles */
master_t masterConfig;
profile_t *currentProfile;
controlRateConfig_t *currentControlRateProfile;
static uint8_t currentControlRateProfileIndex = 0;

void setProfile(uint8_t profileIndex)
{
	currentProfile = &masterConfig.profile[profileIndex];
	currentControlRateProfileIndex = currentProfile->activeRateProfile;
	currentControlRateProfile = &currentProfile->controlRateProfile[currentControlRateProfileIndex];
}

static void resetPidProfile(pidProfile_t *pidProfile)
{
	pidProfile->P8[ROLL] = 44;		// ROLL = 0
	pidProfile->I8[ROLL] = 40;
	pidProfile->D8[ROLL] = 30;
	pidProfile->P8[PITCH] = 58;		// PITCH = 1
	pidProfile->I8[PITCH] = 50;
	pidProfile->D8[PITCH] = 35;
	pidProfile->P8[YAW] = 70;
	pidProfile->I8[YAW] = 45;
	pidProfile->D8[YAW] = 20;
    pidProfile->P8[PIDALT] = 50;
    pidProfile->I8[PIDALT] = 0;
    pidProfile->D8[PIDALT] = 0;
    pidProfile->P8[PIDPOS] = 15;   // POSHOLD_P * 100;
    pidProfile->I8[PIDPOS] = 0;    // POSHOLD_I * 100;
    pidProfile->D8[PIDPOS] = 0;
    pidProfile->P8[PIDPOSR] = 34;  // POSHOLD_RATE_P * 10;
    pidProfile->I8[PIDPOSR] = 14;  // POSHOLD_RATE_I * 100;
    pidProfile->D8[PIDPOSR] = 53;  // POSHOLD_RATE_D * 1000;
    pidProfile->P8[PIDNAVR] = 25;  // NAV_P * 10;
    pidProfile->I8[PIDNAVR] = 33;  // NAV_I * 100;
    pidProfile->D8[PIDNAVR] = 83;  // NAV_D * 1000;
    pidProfile->P8[PIDLEVEL] = 50;
    pidProfile->I8[PIDLEVEL] = 50;
    pidProfile->D8[PIDLEVEL] = 100;
    pidProfile->P8[PIDMAG] = 40;
    pidProfile->P8[PIDVEL] = 55;
    pidProfile->I8[PIDVEL] = 55;
    pidProfile->D8[PIDVEL] = 75;
	
	pidProfile->pidSumLimit = PIDSUM_LIMIT;
	pidProfile->pidSumLimitYaw = PIDSUM_LIMIT_YAW;
	pidProfile->yaw_lpf_hz = 0;
	pidProfile->itermWindupPointPercent = 50;
	pidProfile->dterm_filter_type = FILTER_BIQUAD;			// FILTER_BIQUAD = 1
	pidProfile->dterm_lpf_hz = 100;							// filtering on by default
	pidProfile->dterm_notch_hz = 260;
	pidProfile->dterm_notch_cutoff = 160;
	pidProfile->vbatPidCompensation = 0;
	pidProfile->pidAtMinThrottle = PID_STABILISATION_ON;	// PID_STABILISATION_ON = 1, PID_STABILISATION_OFF = 0
	pidProfile->levelAngleLimit = 55;
	pidProfile->levelSensitivity = 55;
	pidProfile->setpointRelaxRatio = 100;
	pidProfile->dtermSetpointWeight = 60;
	pidProfile->yawRateAccelLimit = 10.0f;
	pidProfile->rateAccelLimit = 0.0f;
	pidProfile->itermThrottleThreshold = 350;
	pidProfile->itermAcceleratorGain = 3.0f;
}

static void resetControlRateConfig(controlRateConfig_t *controlRateConfig)
{
	controlRateConfig->rcRate8 = 100;
	controlRateConfig->rcYawRate8 = 100;
//	controlRateConfig->rcExpo8 = 20;					// Just for testing
	controlRateConfig->rcExpo8 = 0;
	controlRateConfig->thrMid8 = 50;
	controlRateConfig->thrExpo8 = 0;
	controlRateConfig->dynThrPID = 10;					// 10 / 100 = 0.10 (TPA percentage value in BF)
//	controlRateConfig->rcYawExpo8 = 20;					// Just for testing
	controlRateConfig->rcYawExpo8 = 0;
	controlRateConfig->tpa_breakpoint = 1650;
	
	for (uint8_t axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++) {
		controlRateConfig->rates[axis] = 70;
	}
}

void resetProfile(profile_t *profile)
{
	resetPidProfile(&profile->pidProfile);
	
	/* MAX_RATEPROFILES = 3 */
	for (int rateIndex = 0; rateIndex < MAX_RATEPROFILES; rateIndex++) {
		resetControlRateConfig(&profile->controlRateProfile[rateIndex]);
	}
	
	profile->activeRateProfile = 0;			// set activeRateProfile to be the first one
}

void ResetSerialPinConfig(serialPinConfig_t *pSerialPinConfig)
{
	for (int port = 0; port < SERIAL_PORT_MAX_INDEX; port++) {
		pSerialPinConfig->ioTagRx[port] = IO_TAG(NONE);
		pSerialPinConfig->ioTagTx[port] = IO_TAG(NONE);
	}
	
	for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
		switch (serialPortIdentifiers[index]) {
			case SERIAL_PORT_USART1:
#ifdef USE_UART1
				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART1)] = IO_TAG(UART1_RX_PIN);
				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART1)] = IO_TAG(UART1_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART2:
#ifdef USE_UART2
//				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART2)] = IO_TAG(UART2_RX_PIN);
//				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART2)] = IO_TAG(UART2_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART3:
#ifdef USE_UART3
				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART3)] = IO_TAG(UART3_RX_PIN);
				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART3)] = IO_TAG(UART3_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART4:
				break;

			case SERIAL_PORT_USART5:
				break;

			case SERIAL_PORT_USART6:
#ifdef USE_UART6
				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART6)] = IO_TAG(UART6_RX_PIN);
				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART6)] = IO_TAG(UART6_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART7:
				break;

			case SERIAL_PORT_USART8:
				break;

			case SERIAL_PORT_SOFTSERIAL1:
				break;

			case SERIAL_PORT_SOFTSERIAL2:
				break;
			
			case SERIAL_PORT_USB_VCP:
				break;
			
			case SERIAL_PORT_NONE:
				break;
		}
	}
}

void ResetSerialConfig(serialConfig_t *serialConfig)
{
	memset(serialConfig, 0, sizeof(serialConfig_t));
	serialConfig->serial_update_rate_hz = 100;
	serialConfig->reboot_character		= 'R';
	
	for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
		serialConfig->portConfigs[index].identifier			= serialPortIdentifiers[index];
		serialConfig->portConfigs[index].msp_baudrateIndex	= BAUD_115200;
		serialConfig->portConfigs[index].gps_baudrateIndex	= BAUD_115200;
//		serialConfig->portConfigs[index].gps_baudrateIndex	= BAUD_57600;
		serialConfig->portConfigs[index].blackbox_baudrateIndex	= BAUD_115200;		// blackbox port for debugging purposes
		serialConfig->portConfigs[index].telemetry_baudrateIndex = BAUD_AUTO;
	}
	
	serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;
}

void ResetLedStatusConfig(LedStatusConfig_t *ledStatusConfig)
{
	for (int i = 0; i < LED_NUMBER; i++) {
		ledStatusConfig->ledTags[i] = IO_TAG_NONE;
	}
	
#ifdef LED3
	ledStatusConfig->ledTags[0] = IO_TAG(LED3);	// LED3 = PD13 ==> DEFIO_TAG__PD13 ==> 4D
#endif

#ifdef LED4
	ledStatusConfig->ledTags[1] = IO_TAG(LED4);	// LED4 = PD12 ==> DEFIO_TAG__PD12 ==> 4C
#endif

#ifdef LED5
	ledStatusConfig->ledTags[2] = IO_TAG(LED5);	// LED5 = PD14 ==> DEFIO_TAG__PD14 ==> 4E
#endif

#ifdef LED6
	ledStatusConfig->ledTags[3] = IO_TAG(LED6);	// LED6 = PD15 ==> DEFIO_TAG__PD15 ==> 4F
#endif
	
	ledStatusConfig->polarity = 0;
}

void ResetLedTimerConfig(LedTimerConfig_t *ledTimerConfig)
{
	int ledTimerIndex = 0;
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && ledTimerIndex < LED_NUMBER; i++) {
		if (timerHardware[i].usageFlags & TIM_USE_LED) {
			ledTimerConfig->ioTags[ledTimerIndex] = timerHardware[i].tag;
			ledTimerIndex++;
		}
	}
}

void ResetMixerConfig(mixerConfig_t *mixerConfig)
{
	mixerConfig->mixerMode = MIXER_QUADX;
	mixerConfig->yaw_motor_direction = 1;
}

void ResetMotorConfig(motorConfig_t *motorConfig)
{
	motorConfig->minthrottle = 1045;
	motorConfig->motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;		// BRUSHLESS_MOTORS_PWM_RATE = 480 (could be 1K, 2K, 4K, and 8K depending on the PID looptime)
	motorConfig->motorPwmProtocol = PWM_TYPE_STANDARD;			// could be starting with PWM_TYPE_ONESHOT125
	motorConfig->maxthrottle = 2000;
	motorConfig->mincommand = 1000;
//	motorConfig->digitalIdleOffsetPercent = 4.5f;				// this is for MOTOR DSHOT DMA protocol
	
	motorConfig->useUnsyncedPwm = false; // not using synced PWM rate for motors, which means the motors pwm signals are updated at the end of each PID looptime
//	motorConfig->useUnsyncedPwm = true; // using unsynced PWM rate for motors, which means the motors pwm signals are updated apart from each PID looptime
	
	/* store TIMER MOTOR I/O TAG to motorConfig ioTags */
	int motorIndex = 0;
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && motorIndex < QUAD_MOTOR_COUNT; i++) {
//	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && motorIndex < MAX_SUPPORTED_MOTORS; i++) {
		if (timerHardware[i].usageFlags & TIM_USE_MOTOR) {
			motorConfig->ioTags[motorIndex] = timerHardware[i].tag;
			motorIndex++;
		}
	}
}

#if defined(USE_PWM)		// PWM rx signals
void ResetPwmConfig(pwmConfig_t *pwmConfig)
{
	int inputIndex = 0;
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && inputIndex < PWM_INPUT_PORT_COUNT; i++) {
		if (timerHardware[i].usageFlags & TIM_USE_PWM) {
			pwmConfig->ioTags[inputIndex] = timerHardware[i].tag;
			inputIndex++;
		}
	}
}
#endif

/*
 * serialPortIdentifiers[0] = USB_VCP
 * serialPortIdentifiers[1] = USART1
 * serialPortIdentifiers[2] = USART2
 * serialPortIdentifiers[3] = USART3
 * serialPortIdentifiers[4] = USART4
 * serialPortIdentifiers[5] = USART5
 * serialPortIdentifiers[6] = USART6
 * serialPortIdentifiers[7] = USART7
 * serialPortIdentifiers[8] = USART8
 * serialPortIdentifiers[9] = SOFTSERIAL1
 * serialPortIdentifiers[10] = SOFTSERIAL2
 */
void targetConfiguration(master_t *config)
{
	int index;
	
	/* USART1 for MSP */
	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART1);			// index = 0 (SERIAL_PORT_USART1)
	config->serialConfig.portConfigs[index].functionMask = FUNCTION_MSP;

	/* USART2 */
//	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART2);
//	config->serialConfig.portConfigs[index].functionMask = FUNCTION_GPS;
	
	/* USART3 */
	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART3);		// index = 1 (serialPortIdentifiers[1] contains SERIAL_PORT_USART3 which is 2)
	config->serialConfig.portConfigs[index].functionMask = FUNCTION_BLACKBOX;			// USART3 used for printf debugger JUST FOR TESTING PURPOSE NOW

	/* USART6 */
	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART6);		// index = 2 (serialPortIdentifiers[2] contains SERIAL_PORT_USART6 which is 5)
	config->serialConfig.portConfigs[index].functionMask = FUNCTION_RX_SERIAL;
//	config->serialConfig.portConfigs[index].functionMask = FUNCTION_TELEMETRY_FRSKY;	// USART6 used for FRSKY TELEMETRY
	
	/* Mode activations (HARD-CODED for now)
	 * 
	 * IMPORTANT: fc_msp functions will be implemented later to replace the hard-coded mode activation codes
	 */
	/* Setup the range of ARMING switch, which using AUX1 (L04 and !L04 on Frsky Taranis), auxChannelIndex = 0
	 *
	 * ARMING is switched ON between the range of 900 to 1150.
	 */
	config->modeActivationProfile.modeActivationConditions[0].modeId			= BOXARM;			// BOXARM = 0
	config->modeActivationProfile.modeActivationConditions[0].auxChannelIndex	= AUX1 - NON_AUX_CHANNEL_COUNT;	// AUX1 - NON_AUX_CHANNEL_COUNT = 4 - 4 = 0
	config->modeActivationProfile.modeActivationConditions[0].range.startStep	= CHANNEL_VALUE_TO_STEP(900);
	config->modeActivationProfile.modeActivationConditions[0].range.endStep		= CHANNEL_VALUE_TO_STEP(1150);	// Between 900 and 1150 is the ARMING range

	/* Setup the range of BEEPER switch, which using AUX2 (SB on Frsky Taranis), auxChannelIndex = 1
	 *
	 * BEEPER is switched ON between the range of 900 to 1150.
	 */
	config->modeActivationProfile.modeActivationConditions[1].modeId			= BOXBEEPERON;		// BOXBEEPERON = 13
	config->modeActivationProfile.modeActivationConditions[1].auxChannelIndex	= AUX2 - NON_AUX_CHANNEL_COUNT;	// AUX2 - NON_AUX_CHANNEL_COUNT = 5 - 4 = 1
	config->modeActivationProfile.modeActivationConditions[1].range.startStep	= CHANNEL_VALUE_TO_STEP(900);
	config->modeActivationProfile.modeActivationConditions[1].range.endStep		= CHANNEL_VALUE_TO_STEP(1150);	// Between 900 and 1150 is the BEEPER range
	
	/* Setup the range of FlightMode switch, which using AUX3 (SD on Frsky Taranis), auxChannelIndex = 2
	 *
	 * Channel value between 900 and 1150 is setup to ANGLE mode.
	 * Channel value between 1350 and 1650 is setup to HORIZON mode.
	 * Channel value between 1800 and 2100 is setup to ACRO(RATE) mode.
	 */
	config->modeActivationProfile.modeActivationConditions[2].modeId			= BOXANGLE;			// BOXANGLE = 1
	config->modeActivationProfile.modeActivationConditions[2].auxChannelIndex	= AUX3 - NON_AUX_CHANNEL_COUNT;	// AUX3 - NON_AUX_CHANNEL_COUNT = 6 - 4 = 2
	config->modeActivationProfile.modeActivationConditions[2].range.startStep	= CHANNEL_VALUE_TO_STEP(900);
	config->modeActivationProfile.modeActivationConditions[2].range.endStep		= CHANNEL_VALUE_TO_STEP(1150);	// Between 900 and 1150 is the ANGLEMode range
	
	config->modeActivationProfile.modeActivationConditions[3].modeId			= BOXHORIZON;		// BOXHORIZON = 2
	config->modeActivationProfile.modeActivationConditions[3].auxChannelIndex	= AUX3 - NON_AUX_CHANNEL_COUNT;	// AUX3 - NON_AUX_CHANNEL_COUNT = 6 - 4 = 2
	config->modeActivationProfile.modeActivationConditions[3].range.startStep	= CHANNEL_VALUE_TO_STEP(1350);
	config->modeActivationProfile.modeActivationConditions[3].range.endStep		= CHANNEL_VALUE_TO_STEP(1650);	// Between 1350 and 1650 is the HORIZONMode range
	
	/* Setup the range of AirMode switch, which using AUX4 (SG on Frsky Taranis), auxChannelIndex = 3
	 *
	 * AIRMode is switched ON between the range of 900 to 1150.
	 */
	config->modeActivationProfile.modeActivationConditions[4].modeId			= BOXAIRMODE;		// BOXAIRMODE = 28
	config->modeActivationProfile.modeActivationConditions[4].auxChannelIndex	= AUX4 - NON_AUX_CHANNEL_COUNT;	// AUX4 - NON_AUX_CHANNEL_COUNT = 7 - 4 = 3
	config->modeActivationProfile.modeActivationConditions[4].range.startStep	= CHANNEL_VALUE_TO_STEP(900);
	config->modeActivationProfile.modeActivationConditions[4].range.endStep		= CHANNEL_VALUE_TO_STEP(1150);
}

#ifdef BEEPER
void ResetBeeperConfig(beeperConfig_t *beeperConfig)
{
#ifdef BEEPER_INVERTED
	beeperConfig->isOpenDrain = false;
	beeperConfig->isInverted = true;
#else
	beeperConfig->isOpenDrain = false;			// use IO push-pull, with both transistors connected to the VCC and GND
//	beeperConfig->isOpenDrain = true;			// use IO open-drain, no transistor connected to VCC, only with transistor connected to the GND
	beeperConfig->isInverted = false;
#endif
	beeperConfig->ioTag = IO_TAG(BEEPER);
}
#endif

void resetRcControlsConfig(rcControlsConfig_t *rcControlsConfig)
{
	rcControlsConfig->deadband = 0;
	rcControlsConfig->yaw_deadband = 0;
	rcControlsConfig->alt_hold_deadband = 40;
	rcControlsConfig->alt_hold_fast_change = 1;
}

#ifdef USE_SDCARD
void resetsdcardConfig(sdcardConfig_t *sdcardConfig)
{
#if defined(SDCARD_DMA_CHANNEL_TX)
	sdcardConfig->useDma = true;
#else
	sdcardConfig->useDma = false;
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
#endif
}
#endif

void createDefaultConfig(master_t *config)
{	
	/* Clear all configuration */
	memset(config, 0, sizeof(master_t));
	
	/* Feature configuration */
	uint32_t *featuresPtr = &config->enabledFeatures;
	intFeatureClearAll(featuresPtr);
	intFeatureSet(DEFAULT_RX_FEATURE, featuresPtr);

#ifdef DEFAULT_FEATURES
	intFeatureSet(DEFAULT_FEATURES, featuresPtr);		// Initialise default features including airmode right now.
#endif
	
	/* Setup the debug mode for debugging purposes */
	config->debug_mode = DEBUG_MODE;		// DEBUG_MODE default is DEBUG_NONE
	config->task_statistics = true;
	
	config->current_profile_index = 0;		// default profile number
	
	/* Global settings */
	config->gyroConfig.gyro_lpf = GYRO_LPF_256HZ;			// 256Hz default
#ifdef STM32F10X
	config->gyroConfig.gyro_sync_denom = 8;
	config->pidConfig.pid_process_denom = 1;
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)
	config->gyroConfig.gyro_sync_denom = 1;
    config->pidConfig.pid_process_denom = 4;
#else
	config->gyroConfig.gyro_sync_denom = 4;
	config->pidConfig.pid_process_denom = 2;
#endif	
	config->gyroConfig.gyro_soft_lpf_type = FILTER_PT1;
	config->gyroConfig.gyro_soft_lpf_hz = 90;
	config->gyroConfig.gyro_soft_notch_hz_1 = 400;
	config->gyroConfig.gyro_soft_notch_cutoff_1 = 300;
	config->gyroConfig.gyro_soft_notch_hz_2 = 200;
	config->gyroConfig.gyro_soft_notch_cutoff_2 = 100;
	
	config->gyroConfig.gyro_align = ALIGN_DEFAULT;
	config->accelerometerConfig.acc_align = ALIGN_DEFAULT;
	
	/* This threshold means how much average gyro reading could differ before re-calibration is triggered */
	config->gyroConfig.gyroMovementCalibrationThreshold = 48;		// moron_threshold of CLI, range from 0 to 200
	config->accelerometerConfig.acc_hardware = ACC_DEFAULT;
	config->rcControlsConfig.yaw_control_direction = 1;
	
	ResetSerialPinConfig(&config->serialPinConfig);
	ResetSerialConfig(&config->serialConfig);
	ResetLedStatusConfig(&config->ledStatusConfig);
	ResetLedTimerConfig(&config->ledTimerConfig);
	ResetMixerConfig(&config->mixerConfig);
	ResetMotorConfig(&config->motorConfig);
	
	/* custom mixer, clear by defaults */
	for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
		config->customMotorMixer[i].throttle = 0.0f;
	}
    
    ResetRollAndPitchTrims(&config->accelerometerConfig.accelerometerTrims);

#ifdef USE_PWM
	ResetPwmConfig(&config->pwmConfig);
	config->pwmConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;
#endif

	config->rxConfig.serialrx_provider = SERIALRX_SBUS;
	config->rxConfig.halfDuplex = 0;
	config->rxConfig.rx_spi_protocol = 0;			// TODO: 0 for now
	config->rxConfig.sbus_inversion = 1;
	config->rxConfig.spektrum_sat_bind = 0;
	config->rxConfig.spektrum_sat_bind_autoreset = 1;
	config->rxConfig.midrc = 1500;
	config->rxConfig.mincheck = 1100;
	config->rxConfig.maxcheck = 1900;
	config->rxConfig.rx_min_usec = 885;				// any of first 4 channels below this value will trigger rx loss detection
	config->rxConfig.rx_max_usec = 2115;			// any of first 4 channels above this value will trigger rx loss detection

	/* Failsafe initialisation */
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &config->rxConfig.failsafe_channel_configurations[i];
		/* i < NON_AUX_CHANNEL_COUNT means i = 0, 1, 2, 3 which are ROLL, PITCH, THROTTLE, YAW
		 * Related to getRxfailValue() function
		 */
        channelFailsafeConfiguration->mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        channelFailsafeConfiguration->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.midrc);
	}

	config->rxConfig.rssi_channel = 0;
	config->rxConfig.rssi_scale = RSSI_SCALE_DEFAULT;
	config->rxConfig.rssi_invert = 0;

	config->rxConfig.rcInterpolation = RC_SMOOTHING_AUTO;
	config->rxConfig.rcInterpolationChannels = 0;
	config->rxConfig.rcInterpolationInterval = 19;
	config->rxConfig.fpvCamAngleDegrees = 0;
	config->rxConfig.max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT;			// DEFAULT_AUX_CHANNEL_COUNT = 18 - 4 = 14
	
	config->rxConfig.airModeActivateThreshold = 1350;
	
	resetAllRxChannelRangeConfigurations(config->rxConfig.channelRanges);		// set min and max values
	
	config->armingConfig.gyro_cal_on_first_arm = 0;				// TODO: cleanup retarded arm support (allow disarm/arm on throttle down + roll left/right)
	config->armingConfig.disarm_kill_switch = 1;				// allow disarm via AUX switch regardless of throttle value
	config->armingConfig.auto_disarm_delay = 5;					// 5 seconds (allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0.)

//#ifdef RX_CHANNELS_TAER
//	parseRcChannels("TAER1234", &config->rxConfig);
//#else
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
//	parseRcChannels("AETR12", &config->rxConfig);		// 6 channels mapping, A: Roll, E: Pitch, T: Throttle, R: Yaw
	parseRcChannels("AETR1234", &config->rxConfig);		// 8 channels mapping, A: Roll, E: Pitch, T: Throttle, R: Yaw
//#endif

#ifdef USE_SDCARD
	intFeatureSet(FEATURE_SDCARD, featuresPtr);
	resetsdcardConfig(&config->sdcardConfig);
#endif

#ifdef BLACKBOX
#if defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
	intFeatureSet(FEATURE_BLACKBOX, featuresPtr);
	config->blackboxConfig.device = BLACKBOX_DEVICE_SDCARD;		// BLACKBOX using SDCARD media
#endif
	config->blackboxConfig.rate_num = 1;
	config->blackboxConfig.rate_denom = 1;
	config->blackboxConfig.on_motor_test = 0;	// default off
#endif

#ifdef BEEPER
	ResetBeeperConfig(&config->beeperConfig);
#endif

	resetProfile(&config->profile[0]);
	
	resetRcControlsConfig(&config->rcControlsConfig);

	/* merely to force a reset if the person inadvertently flashes the wrong target */
	strncpy(config->boardIdentifier, TARGET_BOARD_IDENTIFIER, sizeof(TARGET_BOARD_IDENTIFIER));

	targetConfiguration(config);
	
	/* copy first profile into remaining profile */
	for (int i = 1; i < MAX_PROFILE_COUNT; i++) {			// MAX_PROFILE_COUNT = 3
		memcpy(&config->profile[i], &config->profile[0], sizeof(profile_t));
	}	
}

static void resetConfig(void)
{
	createDefaultConfig(&masterConfig);
	
	setProfile(0);
}

void resetEEPROM(void)
{
	resetConfig();
}

void checkEEPROMContainsValidData(void)
{
	/* do not check the validity as the configuration does not even write into the EEPROM flash */
//	if (isEEPROMContentValid()) {
//		return;
//	}
	
	resetEEPROM();
}

void activateConfig(void)
{
	generateThrottleCurve();
	
	useRcControlsConfig(ModeActivationProfile()->modeActivationConditions, &masterConfig.motorConfig, &currentProfile->pidProfile);
	
	/* Assign masterConfig.motorConfig, masterConfig.mixerConfig and masterConfig.rxConfig to internal motorConfig, mixerConfig and rxConfig in mixer.c */
	mixerUseConfigs(&masterConfig.motorConfig, &masterConfig.mixerConfig, &masterConfig.rxConfig);
}

void validateAndFixGyroConfig(void)
{
	/* Prevent invalid notch cutoff */
	if (GyroConfig()->gyro_soft_notch_cutoff_1 >= GyroConfig()->gyro_soft_notch_hz_1) {
		GyroConfig()->gyro_soft_notch_hz_1 = 0;
	}
	
	if (GyroConfig()->gyro_soft_notch_cutoff_2 >= GyroConfig()->gyro_soft_notch_hz_2) {
		GyroConfig()->gyro_soft_notch_hz_2 = 0;
	}
	
	float samplingTime = 0.000125f;			// 1 / 8000 (gyro sampling frequency 8K) = 0.000125 
	
	// TODO: implement rest of the gyroConfig stuffs
    
}

void validateAndFixConfig(void)
{
	if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_SERIAL))) {
		featureSet(DEFAULT_RX_FEATURE);
	}
	
	if (featureConfigured(FEATURE_RX_SERIAL)) {
		featureClear(FEATURE_RX_PARALLEL_PWM);
	}
	
	if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
		featureClear(FEATURE_RX_SERIAL);
	}
	
	validateAndFixGyroConfig();		// TODO
}

void beeperOffSet(uint32_t mask)
{
	masterConfig.beeper_off_flags |= mask;
}

void beeperOffSetAll(uint8_t beeperCount)
{
	masterConfig.beeper_off_flags = (1 << beeperCount) - 1;
}

void beeperOffClear(uint32_t mask)
{
	masterConfig.beeper_off_flags &= ~(mask);
}

void beeperOffClearAll(void)
{
	masterConfig.beeper_off_flags = 0;
}

uint32_t getBeeperOffMask(void)
{
	return masterConfig.beeper_off_flags;
}

void setBeeperOffMask(uint32_t mask)
{
	masterConfig.beeper_off_flags = mask;
}

uint32_t getPreferredBeeperOffMask(void)
{
	return masterConfig.preferred_beeper_off_flags;
}

void setPreferredBeeperOffMask(uint32_t mask)
{
	masterConfig.preferred_beeper_off_flags = mask;
}
