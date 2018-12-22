#ifndef __FEATURE_H
#define __FEATURE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#if 0
#ifndef DEFAULT_FEATURES
#define DEFAULT_FEATURES	0
#endif
#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE	FEATURE_RX_PARALLEL_PWM
//#define DEFAULT_RX_FEATURE	FEATURE_RX_SERIAL
#endif
#endif

typedef enum {
    FEATURE_RX_PPM = 1 << 0,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_RX_SERIAL = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_GPS = 1 << 7,
    FEATURE_RANGEFINDER = 1 << 9,
    FEATURE_TELEMETRY = 1 << 10,
    FEATURE_3D = 1 << 12,
    FEATURE_RX_PARALLEL_PWM = 1 << 13,
    FEATURE_RX_MSP = 1 << 14,
    FEATURE_RSSI_ADC = 1 << 15,
    FEATURE_LED_STRIP = 1 << 16,
    FEATURE_DASHBOARD = 1 << 17,
    FEATURE_OSD = 1 << 18,
    FEATURE_CHANNEL_FORWARDING = 1 << 20,
    FEATURE_TRANSPONDER = 1 << 21,
    FEATURE_AIRMODE = 1 << 22,
	FEATURE_SDCARD = 1 << 23,
	FEATURE_BLACKBOX = 1 << 24,
    FEATURE_RX_SPI = 1 << 25,
    FEATURE_SOFTSPI = 1 << 26,
    FEATURE_ESC_SENSOR = 1 << 27,
    FEATURE_ANTI_GRAVITY = 1 << 28,
    FEATURE_DYNAMIC_FILTER = 1 << 29,
} features_e;

void latchActiveFeatures(void);
bool featureConfigured(uint32_t mask);
bool feature(uint32_t mask);
void featureSet(uint32_t mask);
void featureClear(uint32_t mask);
void featureClearAll(void);
uint32_t featureMask(void);

void intFeatureClearAll(uint32_t *features);
void intFeatureSet(uint32_t mask, uint32_t *features);
void intFeatureClear(uint32_t mask, uint32_t *features);

#endif	// __FEATURE_H
