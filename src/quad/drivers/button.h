#ifndef __BUTTON_H
#define __BUTTON_H

#include "exti.h"

#define USER_BUTTON_PIN             PA0

typedef struct buttonDev_s {
	extiCallbackRec_t exti;
	const extiConfig_t *btnIntExtiConfig;
	bool btnPressed;
}buttonDev_t;

typedef struct button_s {
	buttonDev_t dev;
}button_t;

bool buttonInit(void);
void userBtnPollInit(void);
void userBtnPollOps(void);

#endif	// __BUTTON_H
