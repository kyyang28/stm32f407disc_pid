#ifndef __PLATFORM_H
#define __PLATFORM_H

#if defined(STM32F40_41xxx)
#include "stm32f4xx_conf.h"
//#include "stm32f4xx_rcc.h"		// included in stm32f4xx_conf.h
//#include "stm32f4xx_gpio.h"		// included in stm32f4xx_conf.h
//#include "stm32f4xx_tim.h"		// included in stm32f4xx_conf.h
//#include "stm32f4xx_dma.h"		// included in stm32f4xx_conf.h
//#include "stm32f4xx_flash.h"		// included in stm32f4xx_conf.h
#include "core_cm4.h"

// Chip Unique ID on F405
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#define STM32F4

#else
#error "Invalid chipset specified. Update platform.h"
#endif

#include "target.h"

#endif	// __PLATFORM_H
