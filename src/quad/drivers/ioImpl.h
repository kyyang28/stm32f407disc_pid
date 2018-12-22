#ifndef __IOIMPL_H
#define __IOIMPL_H

#include "stm32f4xx.h"
#include "resource.h"

typedef struct ioRec_s {
	GPIO_TypeDef *gpio;
	uint16_t pin;
	resourceOwner_e owner;
	uint8_t index;
}ioRec_t;

#endif	// __IOIMPL_H
