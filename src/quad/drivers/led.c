
#include <stdio.h>

#include "led.h"
#include "utils.h"

static IO_t leds[LED_NUMBER];
static uint8_t ledPolarity = 0;

void LedInit(LedStatusConfig_t *ledStatusConfig)
{
	LED3_OFF;
	LED4_OFF;
	LED5_OFF;
	LED6_OFF;
	
	ledPolarity = ledStatusConfig->polarity;
	for (int i = 0; i < LED_NUMBER; i++) {
		if (ledStatusConfig->ledTags[i]) {
			leds[i] = IOGetByTag(ledStatusConfig->ledTags[i]);
			IOInit(leds[i], OWNER_LED, RESOURCE_INDEX(i));		// no need
			IOConfigGPIO(leds[i], IOCFG_OUT_PP);
		}else {
			leds[i] = IO_NONE;
		}
	}

	LED3_OFF;
	LED4_OFF;
	LED5_OFF;
	LED6_OFF;
}

void LedToggle(int ledNum)
{
	IOToggle(leds[ledNum]);
}

void LedSet(int ledNum, bool ledState)
{
	const bool inverted = (1 << (ledNum) & ledPolarity);		// inverted = 0
	IOWrite(leds[ledNum], ledState ? inverted : !inverted);
}

void taskLed3(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
//    printf("%s, %d\r\n", __FUNCTION__, __LINE__);
    LedToggle(LED3_NUM);
}

void taskLed4(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
//    printf("%s, %d\r\n", __FUNCTION__, __LINE__);
    LedToggle(LED4_NUM);
}

void taskLed5(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
//    printf("%s, %d\r\n", __FUNCTION__, __LINE__);
    LedToggle(LED5_NUM);
}

void taskLed6(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
//    printf("%s, %d\r\n", __FUNCTION__, __LINE__);
    LedToggle(LED6_NUM);
}
