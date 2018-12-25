
#include <stdio.h>			// testing purposes

#include <string.h>
#include "fc_msp.h"
#include "rc_controls.h"
#include "runtime_config.h"

/* This is calculated at startup based on enabled features */
static uint8_t activeBoxIds[CHECKBOX_ITEM_COUNT];

/* This is the number of filled indexes in above array */
static uint8_t activeBoxIdCount = 0;

void initActiveBoxIds(void)
{
	/* Calculate used boxes based on features and filled availableBoxes[] array */
	memset(activeBoxIds, 0xFF, sizeof(activeBoxIds));
	
	activeBoxIdCount = 0;
	activeBoxIds[activeBoxIdCount++] = BOXARM;
	
	if (!feature(FEATURE_AIRMODE)) {
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		activeBoxIds[activeBoxIdCount++] = BOXAIRMODE;
	}
	
	if (!feature(FEATURE_ANTI_GRAVITY)) {
		activeBoxIds[activeBoxIdCount++] = BOXANTIGRAVITY;
	}

	if (sensors(SENSOR_ACC)) {
		activeBoxIds[activeBoxIdCount++] = BOXANGLE;
		activeBoxIds[activeBoxIdCount++] = BOXHORIZON;
		activeBoxIds[activeBoxIdCount++] = BOXHEADFREE;
	}
	
#ifdef BARO
	if (sensors(SENSOR_BARO)) {
		activeBoxIds[activeBoxIdCount++] = BOXBARO;
	}
#endif
	
#ifdef MAG
	if (sensors(SENSOR_MAG)) {
		activeBoxIds[activeBoxIdCount++] = BOXMAG;
		activeBoxIds[activeBoxIdCount++] = BOXHEADADJ;
	}
#endif
	
#ifdef GPS
	if (feature(FEATURE_GPS)) {
		activeBoxIds[activeBoxIdCount++] = BOXGPSHOME;
		activeBoxIds[activeBoxIdCount++] = BOXGPSHOLD;
	}
#endif
	
#ifdef SONAR
	if (feature(FEATURE_SONAR)) {
		activeBoxIds[activeBoxIdCount++] = BOXSONAR;
	}
#endif
	
	if (feature(FEATURE_FAILSAFE)) {
		activeBoxIds[activeBoxIdCount++] = BOXFAILSAFE;
	}
	
	activeBoxIds[activeBoxIdCount++] = BOXBEEPERON;
	
#ifdef LED_STRIP
	if (feature(FEATURE_LED_STRIP)) {
		activeBoxIds[activeBoxIdCount++] = BOXLEDLOW;
	}
#endif
	
#ifdef BLACKBOX
	if (feature(FEATURE_BLACKBOX)) {
		activeBoxIds[activeBoxIdCount++] = BOXBLACKBOX;
	}
#endif
	
	activeBoxIds[activeBoxIdCount++] = BOXFPVANGLEMIX;
	
	if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
		activeBoxIds[activeBoxIdCount++] = BOXCALIB;
	}
	
	activeBoxIds[activeBoxIdCount++] = BOXOSD;
	
#ifdef TELEMETRY
	if (feature(FEATURE_TELEMETRY) && TelemetryConfig()->telemetry_switch) {
		activeBoxIds[activeBoxIdCount++] = BOXTELEMETRY;
	}
#endif
	
//	for (int i = 0; i < sizeof(activeBoxIds)/sizeof(activeBoxIds[0]); i++) {
//		if (activeBoxIds[i] != 255) {
//			printf("activeBoxIds[%d]: %u\r\n", i, activeBoxIds[i]);
//		}
//	}
}

void mspFcInit(void)
{
	initActiveBoxIds();
}
