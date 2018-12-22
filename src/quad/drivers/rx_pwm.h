#ifndef __RX_PWM_H
#define __RX_PWM_H

#include "io.h"

#define PPM_RCVR_TIMEOUT            	(0u)
#define PWM_INPUT_PORT_COUNT			(8u)

typedef enum {
    INPUT_FILTERING_DISABLED = 0,
    INPUT_FILTERING_ENABLED
} inputFilteringMode_e;

typedef struct pwmConfig_s {
	ioTag_t ioTags[PWM_INPUT_PORT_COUNT];
	inputFilteringMode_e inputFilteringMode;
}pwmConfig_t;

void pwmRxInit(const pwmConfig_t *pwmConfig);
uint16_t pwmRead(uint8_t channel);
//uint32_t pwmRead(uint8_t channel);		// TODO: change back to return type uint16_t later
bool isPWMDataBeingReceived(void);

#endif	// __RX_PWM_H
