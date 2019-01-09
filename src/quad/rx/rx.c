
#include <string.h>
//#include "rx.h"
#include "pwm.h"				// including rx.h
#include "rx_pwm.h"
#include "sbus.h"
#include "utils.h"
#include "system.h"
#include "target.h"
#include "maths.h"

#include <stdio.h>		// debugging purposes, remove later

#define MAX_INVALID_PULS_TIME			300

#define DELAY_10_HZ						(1000000 / 10)				// in usec  100 ms
#define DELAY_50_HZ						(1000000 / 50)				// in usec  20 ms
#define DELAY_1_HZ						(1000000 / 1)				// in usec, JUST FOR TESTING

#define REQUIRED_CHANNEL_MASK			0x0F						// first 4 channels

#define PPM_AND_PWM_SAMPLE_COUNT		3

const char rcChannelLetters[] = "AERT12345678abcdefgh";

uint16_t rssi = 0;													// range: [0;1023]

static bool rxDataReceived = false;
static bool rxSignalReceived = false;
static bool rxSignalReceivedNotDataDriven = false;
static bool rxFlightChannelsValid = false;
static bool rxIsInFailsafeMode = true;
static bool rxIsInFailsafeModeNotDataDriven = true;

static uint32_t rxUpdateAt = 0;
static uint32_t needRxSignalBefore = 0;
static uint32_t needRxSignalMaxDelayUs;
static uint8_t skipRxSamples = 0;
static uint32_t suspendRxSignalUntil = 0;

static const rxConfig_t *rxConfig;
rxRuntimeConfig_t rxRuntimeConfig;
static uint8_t rcSampleIndex = 0;

int16_t rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];			// interval [1000;2000]
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];			// interval [1000;2000]
uint32_t rcInvalidPulsPeriod[MAX_SUPPORTED_RC_CHANNEL_COUNT];

static uint8_t validFlightChannelMask;

void useRxConfig(const rxConfig_t *rxConfigToUse)
{
	rxConfig = rxConfigToUse;
}

static uint16_t nullReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
	UNUSED(rxRuntimeConfig);
	UNUSED(channel);
	
	return 0;			// PPM_RCVR_TIMEOUT = 0
}

static uint8_t nullFrameStatus(void)
{
	return RX_FRAME_PENDING;
}

#ifdef SERIAL_RX
bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	bool enabled = false;
	
	switch (rxConfig->serialrx_provider) {
#ifdef USE_SERIALRX_SBUS
		case SERIALRX_SBUS:
			enabled = sbusInit(rxConfig, rxRuntimeConfig);
			break;
		
		default:
			enabled = false;
#endif
	}
	
	return enabled;
}
#endif

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfiguration_t *rxChannelRangeConfiguration)
{
	/* set default calibration to full range and 1:1 mapping */
	for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
		rxChannelRangeConfiguration->min = PWM_RANGE_MIN;
		rxChannelRangeConfiguration->max = PWM_RANGE_MAX;
		rxChannelRangeConfiguration++;
	}
}

void rxInit(const rxConfig_t *rxConfig, const modeActivationCondition_t *modeActivationConditions)
{
	useRxConfig(rxConfig);
	rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
	rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
	rcSampleIndex = 0;
	needRxSignalMaxDelayUs = DELAY_10_HZ;               // 100 ms
	
	for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
		rcData[i] = rxConfig->midrc;			// midrc = 1500
		rcInvalidPulsPeriod[i] = millis() + MAX_INVALID_PULS_TIME;
	}
	
	/* THROTTLE = 3
	 * rxConfig->midrc = 1500
	 * rxConfig->rx_min_usec = 885
	 *
	 * rcData[THROTTLE] = rxConfig->rx_min_usec = 885 for quadcopters
	 */
	rcData[THROTTLE] = rxConfig->rx_min_usec;
//	rcData[THROTTLE] = (feature(FEATURE_3D)) ? rxConfig->midrc : rxConfig->rx_min_usec;
	
	/* Initialise ARM switch to OFF position when arming via switch is defined */
	for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
		const modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[i];
		if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
			/* ARM switch is defined, determine an OFF value */
			uint16_t value;
			if (modeActivationCondition->range.startStep > 0) {
				value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.startStep - 1));
			}else {
				value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.endStep + 1));
			}
			
			/* Initialise ARM AUX channel to OFF value */
			rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
		}
	}
	
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		
#ifdef SERIAL_RX
	if (feature(FEATURE_RX_SERIAL)) {
//		printf("FEATURE_RX_SERIAL: %s, %d\r\n", __FUNCTION__, __LINE__);
		const bool enabled = serialRxInit(rxConfig, &rxRuntimeConfig);
		if (!enabled) {
			printf("%s, %d\r\n", __FUNCTION__, __LINE__);
			featureClear(FEATURE_RX_SERIAL);
			rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
			rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
		}
	}
#endif

#if defined(USE_PWM)
	if (feature(FEATURE_RX_PARALLEL_PWM)) {
//		printf("FEATURE_RX_PARALLEL_PWM: %s, %d\r\n", __FUNCTION__, __LINE__);
		rxPwmInit(rxConfig, &rxRuntimeConfig);
	}
#endif
}

/* TODO: rxUpdateCheck() runction will be running in a separate RTOS task */
bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime)
{
	UNUSED(currentDeltaTime);
	
	if (rxSignalReceived) {
//        printf("%u, %u: %d\r\n", currentTimeUs, needRxSignalBefore);
		if (currentTimeUs >= needRxSignalBefore) {
			rxSignalReceived = false;
			rxSignalReceivedNotDataDriven = false;
//            printf("%d\r\n", __LINE__);
		}
	}
	
#if defined(USE_PWM)
	if (feature(FEATURE_RX_PARALLEL_PWM)) {
		if (isPWMDataBeingReceived()) {
			rxSignalReceivedNotDataDriven = true;
			rxIsInFailsafeModeNotDataDriven = false;
			needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
		}
	}else
#endif
	{
		rxDataReceived = false;
		const uint8_t frameStatus = rxRuntimeConfig.rcFrameStatusFn();
//        printf("frameStatus: %u\r\n", frameStatus);
		if (frameStatus & RX_FRAME_COMPLETE) {
			rxDataReceived = true;
			rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
//            printf("rxIsInFailsafeMode: %d\r\n", rxIsInFailsafeMode);
			rxSignalReceived = !rxIsInFailsafeMode;
//            printf("rxSignalReceived: %d\r\n", rxSignalReceived);
			needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
		}
	}
	
//    printf("%d-\r\n", rxDataReceived);
//    printf("%d--\r\n", currentTimeUs >= rxUpdateAt);
//    printf("%d\r\n", currentTimeUs);
//    printf("%d\r\n", rxUpdateAt);
    
	return rxDataReceived || (currentTimeUs >= rxUpdateAt);		// data driven or 50 Hz
}

static void rxResetFlightChannelStatus(void)
{
	validFlightChannelMask = REQUIRED_CHANNEL_MASK;
}

static uint8_t getRxChannelCount(void)
{
	static uint8_t maxChannelsAllowed;
	
	if (!maxChannelsAllowed) {
		uint8_t maxChannels = rxConfig->max_aux_channel + NON_AUX_CHANNEL_COUNT;	// max_aux_channel = 18 - 4 = 14; NON_AUX_CHANNEL_COUNT = 4, so maxChannels = 18
		if (maxChannels > rxRuntimeConfig.channelCount) {  // rxRuntimeConfig.channelCount = 6 for PWM inputs and rxRuntimeConfig.channelCount = SBUS_MAX_CHANNEL = 18 for SBUS
			maxChannelsAllowed = rxRuntimeConfig.channelCount;  // maxChannelsAllowed = rxRuntimeConfig.channelCount = 6 for PWM inputs
		}else {
			maxChannelsAllowed = maxChannels;	// maxChannelsAllowed = maxChannels = 18 for SBUS
		}
	}
	
//	printf("maxChannelsAllowed: %u\r\n", maxChannelsAllowed);	// 0 if USE_PWM is not defined or SERIAL_RX is not defined
	return maxChannelsAllowed;
}

static uint8_t calculateChannelRemapping(const uint8_t *channelMap, uint8_t channelMapEntryCount, uint8_t channelToRemap)
{
	if (channelToRemap < channelMapEntryCount) {
		return channelMap[channelToRemap];
	}
	return channelToRemap;
}

static uint16_t applyRxChannelRangeConfiguration(int sample, rxChannelRangeConfiguration_t range)
{
	/* avoid corruption of channel with a value of PPM_RCVR_TIMEOUT = 0 */
	if (sample == PPM_RCVR_TIMEOUT) {
		return PPM_RCVR_TIMEOUT;
	}
	
	sample = scaleRange(sample, range.min, range.max, PWM_RANGE_MIN, PWM_RANGE_MAX);
	sample = MIN(MAX(PWM_PULSE_MIN, sample), PWM_PULSE_MAX);
	
	return sample;
}

static void readRxChannelsApplyRanges(void)
{
	const int channelCount = getRxChannelCount();			// channelCount = 6 for PWM inputs or channelCount = 18 for SBUS
//	printf("channelCount: %d, %s, %d\r\n", channelCount, __FUNCTION__, __LINE__);
	
#if 0
	/* print out the contents of rcmap array assigned from parseRcChannels("AETR1234", &config->rxConfig) in config.c */
	for (int i = 0; i < REMAPPABLE_CHANNEL_COUNT; i++) {
		printf("rxConfig->rcmap[i]: %d\r\n", rxConfig->rcmap[i]);
	}
#endif
	
	for (int channel = 0; channel < channelCount; channel++) {
//		printf("REMAPPABLE_CHANNEL_COUNT: %d, %s, %d\r\n", REMAPPABLE_CHANNEL_COUNT, __FUNCTION__, __LINE__);
		const uint8_t rawChannel = calculateChannelRemapping(rxConfig->rcmap, REMAPPABLE_CHANNEL_COUNT, channel);	// REMAPPABLE_CHANNEL_COUNT = 8
//		printf("rawChannel: %u, %s, %d\r\n", rawChannel, __FUNCTION__, __LINE__);		// rawChannel = 0(A), 1(E), 3(R), 2(T), 4(1), 5(2), 6(3), 7(4)
		
		/* sample the channel */
		uint16_t sample = rxRuntimeConfig.rcReadRawFn(&rxRuntimeConfig, rawChannel);
		/*
		 * channel 0 (Throttle stick): 996, readRxChannelsApplyRanges, 172
		 * channel 1 (Roll stick): 1501, readRxChannelsApplyRanges, 172
		 * channel 3 (Yaw stick): 1501, readRxChannelsApplyRanges, 172
		 * channel 2 (Pitch stick): 1501, readRxChannelsApplyRanges, 172
		 */
//		printf("channel %d: %u\r\n", rawChannel, sample);
		
		/* apply the rx calibration */
		if (channel < NON_AUX_CHANNEL_COUNT) {
//			printf("range.min: %u, %s, %d\r\n", rxConfig->channelRanges[channel].min, __FUNCTION__, __LINE__);
//			printf("range.max: %u, %s, %d\r\n", rxConfig->channelRanges[channel].max, __FUNCTION__, __LINE__);
//			printf("PWM_RANGE_MIN: %u, %s, %d\r\n", PWM_RANGE_MIN, __FUNCTION__, __LINE__);
//			printf("PWM_RANGE_MAX: %u, %s, %d\r\n", PWM_RANGE_MAX, __FUNCTION__, __LINE__);
			sample = applyRxChannelRangeConfiguration(sample, rxConfig->channelRanges[channel]);
		}
		
		/* store sampled rx value to rcRaw array */
		rcRaw[channel] = sample;

#if 0
		if (channel == 3)	// Channel 3: Throttle
			printf("rcRaw[%d](Throttle stick): %u\r\n", channel, rcRaw[channel]);
#endif

#if 0
		if (channel == 0)		// Channel 1: Roll
			printf("rcRaw[%d](Roll[Ail] stick): %u\r\n", channel, rcRaw[channel]);
		else if (channel == 1)	// Channel 2: Pitch
			printf("rcRaw[%d](Pitch[Ele] stick): %u\r\n", channel, rcRaw[channel]);
		else if (channel == 2)	// Channel 4: Yaw
			printf("rcRaw[%d](Yaw[Rud] stick): %u\r\n", channel, rcRaw[channel]);
		else if (channel == 3)	// Channel 3: Throttle
			printf("rcRaw[%d](Throttle stick): %u\r\n", channel, rcRaw[channel]);
#endif

#if 0
		if (channel == 4)	// Channel 5: BOXARM (L04, logical switch) AUX1
			printf("rcRaw[%d](BOXARM switch (L04)): %u\r\n", channel, rcRaw[channel]);
		else if (channel == 5)	// Channel 6: Switch SB for BEEPER	AUX2
			printf("rcRaw[%d](BEEPER switch (SB)): %u\r\n", channel, rcRaw[channel]);
		else if (channel == 6)	// Channel 7: Switch SD for FLIGHT MODE	AUX3
			printf("rcRaw[%d](FLIGHT MODE switch (SD)): %u\r\n", channel, rcRaw[channel]);
		else if (channel == 7) { // Channel 8: Switch SG for AIR MODE	AUX4
			printf("rcRaw[%d](AIR MODE switch (SG)): %u\r\n", channel, rcRaw[channel]);
			printf("\r\n");
		}
#endif
	}
}

static bool isRxDataDriven(void)
{
	return !feature(FEATURE_RX_PARALLEL_PWM);
}

bool rxIsReceivingSignal(void)
{
	return rxSignalReceived;
}

static bool isPulseValid(uint16_t pulseDuration)
{
	/* rx_min_usec = 885, rx_max_usec = 2115 */
	return pulseDuration >= rxConfig->rx_min_usec && pulseDuration <= rxConfig->rx_max_usec;
}

static void rxUpdateFlightChannelStatus(uint8_t channel, bool valid)
{
//	printf("valid: %d\r\n", valid);
	if (channel < NON_AUX_CHANNEL_COUNT && !valid) {
		/* if signal is invalid - mark channel as BAD */
		validFlightChannelMask &= ~(1 << channel);
	}
}

static uint16_t getRxFailValue(uint8_t channel)
{
	const rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &rxConfig->failsafe_channel_configurations[channel];
	
	switch (channelFailsafeConfiguration->mode) {
		case RX_FAILSAFE_MODE_AUTO:
			switch (channel) {
				case ROLL:
				case PITCH:
				case YAW:
					return rxConfig->midrc;				// midrc = 1500
				case THROTTLE:
					return rxConfig->rx_min_usec;		// rx_min_usec = 885
			}
			/* no break */
		
		default:
		case RX_FAILSAFE_MODE_INVALID:
		case RX_FAILSAFE_MODE_HOLD:
			return rcData[channel];
		
		case RX_FAILSAFE_MODE_SET:
			return RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfiguration->step);
	}
}

static uint16_t calculateNonDataDrivenChannel(uint8_t chan, uint16_t sample)
{
	/* MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT = 8 */
	static int16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];	// 8 x 3 2D-array
	static int16_t rcDataMean[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT];
	static bool rxSamplesCollected = false;
	
	const uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;
	
	/* update the recent samples and compute the average of them */
	rcSamples[chan][currentSampleIndex] = sample;
	
	/* avoid returning an incorrect average which would otherwise occur before enough samples */
	if (!rxSamplesCollected) {
		if (rcSampleIndex < PPM_AND_PWM_SAMPLE_COUNT) {
			return sample;
		}
		rxSamplesCollected = true;
	}
	
	rcDataMean[chan] = 0;

	for (int sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++) {
		rcDataMean[chan] += rcSamples[chan][sampleIndex];
	}
	
	return rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
}

static bool rxHaveValidFlightChannels(void)
{
	return (validFlightChannelMask == REQUIRED_CHANNEL_MASK);
}

static void detectAndApplySignalLossBehaviour(timeUs_t currentTimeUs)
{
	bool useValueFromRx = true;
	const bool rxIsDataDriven = isRxDataDriven();			// rxIsDataDriven = false for PWM input, true for SBUS;
//	printf("rxIsDataDriven: %d, %s, %d\r\n", rxIsDataDriven, __FUNCTION__, __LINE__);
	const uint32_t currentMilliTime = currentTimeUs / 1000;
	
	/* rxIsDataDriven will be always false if using PWM signals of the receiver module */
	if (!rxIsDataDriven) {
		/* rxSignalReceivedNotDataDriven and rxIsInFailsafeModeNotDataDriven variables will be updated in rxUpdateCheck() function
		 * rxUpdateCheck() function will be running in a separate RTOS task
		 */
		rxSignalReceived = rxSignalReceivedNotDataDriven;		// rxSignalReceivedNotDataDriven initial false
		rxIsInFailsafeMode = rxIsInFailsafeModeNotDataDriven;	// rxIsInFailsafeModeNotDataDriven initial true
	}
	
	/* If the FrSKY transmitter is not powered on, the rxSignalReceived = false and rxIsInFailsafeMode = true, then useValueFromRx sets to false */
	if (!rxSignalReceived || rxIsInFailsafeMode) {
		useValueFromRx = false;
	}
	
	/* 0 when the receiver does not receive anything, 1 when the receiver is linked to the transmitter and retrieve the sticks and switches data */
//	printf("useValueFromRx: %d, %s, %d\r\n", useValueFromRx, __FUNCTION__, __LINE__);
	
	/* Reset flight channel status to be valid (0x0F), lower 4 bits are roll, pitch, throttle and yaw stick indicators.
	 * 0x0F (00001111), 4 one's means roll, pitch, throttle and yaw contain the valid stick values
	 */
	rxResetFlightChannelStatus();
	
	/* getRxChannelCount() returns 8 for PWM inputs and 18 for SBUS */
	for (int channel = 0; channel < getRxChannelCount(); channel++) {
		uint16_t sample = (useValueFromRx) ? rcRaw[channel] : PPM_RCVR_TIMEOUT;
		
		/* determine if the sample is valid or not, valid sample from rcRaw[channel], otherwise 0 will be stored in sample variable */
		bool validPulse = isPulseValid(sample);
//		printf("validPulse: %d\r\n", validPulse);
		
		if (!validPulse) {
			/* handle invalid rx input pulse
			 * validFlightChannelMask will be set to zero for each channel in rxUpdateFlightChannelStatus(channel, validPulse) function
			 */
			if (currentMilliTime < rcInvalidPulsPeriod[channel]) {
				sample = rcData[channel];					// hold channel for MAX_INVALID_PULS_TIME
//				printf("sample: %u, %s, %d\r\n", sample, __FUNCTION__, __LINE__);
			}else {
				sample = getRxFailValue(channel);			// after that apply rxfail value
//				printf("sample(rxFailValue): %u\r\n", sample);
				rxUpdateFlightChannelStatus(channel, validPulse);
			}
		}else {
			rcInvalidPulsPeriod[channel] = currentMilliTime + MAX_INVALID_PULS_TIME;
//			printf("rcInvalidPulsPeriod[%d]: %u, %s, %d\r\n", channel, rcInvalidPulsPeriod[channel], __FUNCTION__, __LINE__);
		}
		
		/* rxIsDataDriven = false */
		if (rxIsDataDriven) {
			rcData[channel] = sample;
		}else {
			rcData[channel] = calculateNonDataDrivenChannel(channel, sample);
//			printf("rcData[%d]: %u, %s, %d\r\n", channel, rcData[channel], __FUNCTION__, __LINE__);
		}
	}
//	printf("\r\n");
	
	/* check if roll, pitch, throttle, yaw channel still contain the valid signal, rxFlightChannelsValid should return TRUE */
	rxFlightChannelsValid = rxHaveValidFlightChannels();
//	printf("rxFlightChannelsValid: %d, %s, %d\r\n", rxFlightChannelsValid, __FUNCTION__, __LINE__);

	/* TODO: failsafe implentation later with digestion of RX data */
#if 0	
	if ((rxFlightChannelsValid) && !IS_RC_MODE_ACTIVE(BOXFAILSAFE)) {
		failsafeOnValidDataReceived();
	}else {
		rxIsInFailsafeMode = rxIsInFailsafeModeNotDataDriven = true;
		failsafeOnValidDataFailed();
		
		for (int channel = 0; channel < getRxChannelCount(); channel++) {
			rcData[channel] = getRxfailValue(channel);
		}
	}
#endif	
}

void calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
	rxUpdateAt = currentTimeUs + DELAY_50_HZ;		// delay 20 ms
//	rxUpdateAt = currentTimeUs + DELAY_1_HZ;		// delay 1 second, JUST FOR TESTING
	
	/* only proceed when no more samples to skip and suspend period is over */
	if (skipRxSamples) {
		/* suspendRxSignalUntil is something to do with suspendRxSignal and resumeRxSignal */
		if (currentTimeUs > suspendRxSignalUntil) {
			skipRxSamples--;
		}
		return;
	}
	
	readRxChannelsApplyRanges();
	detectAndApplySignalLossBehaviour(currentTimeUs);
	
	rcSampleIndex++;
}

void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);		// cannot add printf here, the whole program will be stucked. Need to figure out the reason behind this!!!
	const char *c, *s;
	
	for (c = input; *c; c++) {
		s = strchr(rcChannelLetters, *c);
		if (s && (s < rcChannelLetters + MAX_MAPPABLE_RX_INPUTS)) {
			rxConfig->rcmap[s - rcChannelLetters] =  c - input;
		}
	}
}

uint16_t rxGetRefreshRate(void)
{
	return rxRuntimeConfig.rxRefreshRate;
}
