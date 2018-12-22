#ifndef __FC_TASKS_H
#define __FC_TASKS_H

#include "time.h"

void taskUpdateRxMain(timeUs_t currentTimeUs);			// TODO: make this function static for rtos tasks assignment
void fcTasksInit(void);

#endif	// __FC_TASKS_H
