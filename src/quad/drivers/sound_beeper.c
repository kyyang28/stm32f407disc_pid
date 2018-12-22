
#include "sound_beeper.h"
#include "utils.h"

#include <stdio.h>

static IO_t beeperIO = DEFIO_IO(NONE);
static bool beeperInverted = false;

void systemBeep(bool onoff)
{
#ifndef BEEPER
	UNUSED(onoff);
#else
	IOWrite(beeperIO, beeperInverted ? onoff : !onoff);		// set beeperInverted = false, then if onoff = 1, !onoff = 0, turn on the beeper, else onoff = 0, !onoff = 1, turn off the beeper
#endif
}

void systemBeeperToggle(void)
{
#ifdef BEEPER
	IOToggle(beeperIO);
#endif
}

void beeperInit(const beeperConfig_t *beeperConfig)
{
//	printf("beeperConfig->ioTag: 0x%x, %s, %d\r\n", beeperConfig->ioTag, __FUNCTION__, __LINE__);
#ifndef BEEPER
		UNUSED(beeperConfig);
#else
	beeperIO = IOGetByTag(beeperConfig->ioTag);
	beeperInverted = beeperConfig->isInverted;
	
//	printf("IO_GPIO(beeperIO): 0x%x\r\n", (uint32_t)IO_GPIO(beeperIO));		// gpio = 0x40020000 (GPIOA)
//	printf("IO_Pin(beeperIO): %u\r\n", IO_Pin(beeperIO));					// pin = 13 = 0x2000 (1 << 13)
	
	if (beeperIO) {
		IOInit(beeperIO, OWNER_BEEPER, 0);
		IOConfigGPIO(beeperIO, beeperConfig->isOpenDrain ? IOCFG_OUT_OD : IOCFG_OUT_PP);
	}
	
	systemBeep(false);
//	systemBeepToggle();
#endif
}
