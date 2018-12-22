#ifndef __RESOURCE_H
#define __RESOURCE_H

typedef enum {
	OWNER_FREE			= 0,
	OWNER_LED,				// 1
	OWNER_BUTTON,			// 2
	OWNER_USB,				// 3
	OWNER_SYSTEM,			// 4
	OWNER_SERIAL_TX,		// 5
	OWNER_SERIAL_RX,		// 6
	OWNER_SPI_CS,			// 7
	OWNER_SPI_SCK,			// 8
	OWNER_SPI_MISO,			// 9
	OWNER_SPI_MOSI,			// 10
	OWNER_MPU_CS,			// 11
	OWNER_TIMER_LED,		// 12
	OWNER_MOTOR,			// 13
	OWNER_PWMINPUT,			// 14	
	OWNER_MPU_EXTI,			// 15
	OWNER_BEEPER,			// 16
	OWNER_SDCARD,			// 17
	OWNER_SDCARD_CS,		// 18
	OWNER_SDCARD_DETECT,	// 19
	OWNER_TOTAL_COUNT
}resourceOwner_e;

#define RESOURCE_INDEX(x)		((x) + 1)
#define RESOURCE_SOFT_OFFSET    10

#endif	// __RESOURCE_H
