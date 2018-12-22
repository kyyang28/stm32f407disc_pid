
#include <stdio.h>			// debugging purposes
#include "pwm.h"
#include "utils.h"
#include "rx_pwm.h"

static uint16_t pwmReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
	UNUSED(rxRuntimeConfig);
	return pwmRead(channel);
}

void rxPwmInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimConfig)
{
//	printf("%s, %d\r\n", __FUNCTION__, __LINE__);
	UNUSED(rxConfig);
	
	rxRuntimConfig->rxRefreshRate = 20000;
	
	/* Configure PWM read function and max number of channels. Serial RX below will override both of these, if SERIAL_RX is enabled */
	if (feature(FEATURE_RX_PARALLEL_PWM)) {
//		printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
		
		/* IMPORTANT: rxRuntimConfig->channelCount must be initialised here for RX_PWM receiver, otherwise the FC won't receive any signal from FrSKY PWM bus
		 *
		 * Notes: The max supported RC parallel pwm channel is 6 currently
		 *    	  Ch1: Ail(Roll)
		 *        Ch2: Ele(Pitch)
		 *    	  Ch3: Throttle
		 *    	  Ch4: Rud(Yaw)
		 *    	  Ch5: BOXARM(AUX1)
		 *    	  Ch6: BEEPER(AUX2)
		 */
		rxRuntimConfig->channelCount = MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT;		// MAX_SUPPORTED_RC_PARALLEL_PWM_CHANNEL_COUNT = 6
		
		rxRuntimConfig->rcReadRawFn = pwmReadRawRC;		// pwmReadRawRC calls pwmRead() from rx_pwm.c, which returns the value of captures[channel]
	}
}
