#ifndef __RX_H
#define __RX_H

#include <stdint.h>
#include <stdbool.h>
#include "rc_controls.h"
#include "feature.h"
#include "time.h"

#define PWM_RANGE_MIN									1000
#define PWM_RANGE_MAX									2000

#define PWM_PULSE_MIN									750			// minimum PWM pulse width which is considered valid
#define PWM_PULSE_MAX									2250		// maximum PWM pulse width which is considered valid

/* 6 channel mapping which are Ail(Roll, CH0), Ele(Pitch, CH1), Throttle(CH2), Rud(Yaw, CH3), BOXARM(AUX1), BEEPER(AUX2) */
#define MAX_MAPPABLE_RX_INPUTS							8

/* 6 supported PWM channel which are Ail(Roll, CH0), Ele(Pitch, CH1), Throttle(CH2), Rud(Yaw, CH3), BOXARM(AUX1), BEEPER(AUX2) */
#define MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT		8

#define MAX_SUPPORTED_RC_CHANNEL_COUNT					18
#define MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT 		MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT

#define NON_AUX_CHANNEL_COUNT							4
#define MAX_AUX_CHANNEL_COUNT							(MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT)

#define RSSI_SCALE_MIN									1
#define RSSI_SCALE_MAX									255
#define RSSI_SCALE_DEFAULT								30

#define RXFAIL_STEP_TO_CHANNEL_VALUE(step)				(PWM_PULSE_MIN + 25 * step)
#define CHANNEL_VALUE_TO_RXFAIL_STEP(channelValue)		((constrain(channelValue, PWM_PULSE_MIN, PWM_PULSE_MAX) - PWM_PULSE_MIN) / 25)

extern int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];       // interval [1000;2000]

typedef enum {
	RX_FRAME_PENDING = 0,
	RX_FRAME_COMPLETE = (1 << 0),
	RX_FRAME_FAILSAFE = (1 << 1)
}rxFrameState_e;

typedef enum {
    SERIALRX_SPEKTRUM1024 = 0,
    SERIALRX_SPEKTRUM2048 = 1,
    SERIALRX_SBUS = 2,
    SERIALRX_SUMD = 3,
    SERIALRX_SUMH = 4,
    SERIALRX_XBUS_MODE_B = 5,
    SERIALRX_XBUS_MODE_B_RJ01 = 6,
    SERIALRX_IBUS = 7,
    SERIALRX_JETIEXBUS = 8,
    SERIALRX_CRSF = 9,
    SERIALRX_SRXL = 10,
}SerialRXType;

typedef enum {
	RX_FAILSAFE_MODE_AUTO = 0,
	RX_FAILSAFE_MODE_HOLD,
	RX_FAILSAFE_MODE_SET,
	RX_FAILSAFE_MODE_INVALID
}rxFailsafeChannelMode_e;

typedef struct rxFailsafeChannelConfiguration_s {
	uint8_t mode;			// see rxFailsafeChannelMode_e
	uint8_t step;
}rxFailsafeChannelConfiguration_t;

typedef struct rxChannelRangeConfiguration_s {
	uint16_t min;
	uint16_t max;
}rxChannelRangeConfiguration_t;

typedef struct rxConfig_s {
	uint8_t rcmap[MAX_MAPPABLE_RX_INPUTS];			// mapping of radio channels to internal RPYTA+ order
	
	uint8_t serialrx_provider;						// type of UART-based receiver (0 = spek, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
	
	uint8_t sbus_inversion;							// default sbus (Futaba, FrSKY) is inverted. Support for uninverted OpenLRS (and modified FrSKY) receivers
	uint8_t halfDuplex;								// allow rx to operate in half duplex mode on F4, ignored for F1 and F3
	
	uint8_t rx_spi_protocol;						// type of nrf24 protocol (0 = v202 250kbps). Must be enabled by FEATURE_RX_NRF24 first.
	uint32_t rx_spi_id;
	uint8_t rx_spi_rf_channel_count;
	
	uint8_t spektrum_sat_bind;						// number of bind pulses for Spektrum satellite receivers
	uint8_t spektrum_sat_bind_autoreset;			// whenever we will reset (exit) binding mode after hard reboot
	
	uint8_t rssi_channel;
	uint8_t rssi_scale;
	uint8_t rssi_invert;
	
	uint16_t midrc;									// some radios have not a neutral point centred on 1500. can be changed here
	uint16_t mincheck;								// minimum rc end
	uint16_t maxcheck;								// maximum rc end
	
	uint8_t rcInterpolation;
	uint8_t rcInterpolationChannels;
	uint8_t rcInterpolationInterval;
	
	uint8_t fpvCamAngleDegrees;						// camera angle to be scaled into rc commands
	uint8_t max_aux_channel;
	uint16_t airModeActivateThreshold;				// throttle setpoint where airmode gets activated
	
	uint16_t rx_min_usec;
	uint16_t rx_max_usec;
	
	rxFailsafeChannelConfiguration_t failsafe_channel_configurations[MAX_SUPPORTED_RC_CHANNEL_COUNT];
	rxChannelRangeConfiguration_t channelRanges[NON_AUX_CHANNEL_COUNT];
}rxConfig_t;

#define REMAPPABLE_CHANNEL_COUNT		(sizeof(((rxConfig_t *)0)->rcmap) / sizeof(((rxConfig_t *)0)->rcmap[0]))

struct rxRuntimeConfig_s;
/* used by receiver driver to return channel data */
typedef uint16_t (*rcReadRawDataFnPtr)(const struct rxRuntimeConfig_s *rxRuntimeConfig, uint8_t chan);
typedef uint8_t (*rcFrameStatusFnPtr)(void);

typedef struct rxRuntimeConfig_s {
	uint8_t channelCount;			// number of RC channels as reported by current input driver
	uint16_t rxRefreshRate;
	rcReadRawDataFnPtr rcReadRawFn;
	rcFrameStatusFnPtr rcFrameStatusFn;
}rxRuntimeConfig_t;

void rxInit(const rxConfig_t *rxConfig, const modeActivationCondition_t *modeActivationConditions);
void resetAllRxChannelRangeConfigurations(rxChannelRangeConfiguration_t *rxChannelRangeConfiguration);
void calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs);

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime);

void parseRcChannels(const char *input, rxConfig_t *rxConfig);

#endif	// __RX_H
