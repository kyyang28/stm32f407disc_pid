#ifndef __EXTI_H
#define __EXTI_H

#include "IOTypes.h"
#include "target.h"

typedef struct extiConfig_s {
	ioTag_t tag;
}extiConfig_t;

typedef struct extiCallbackRec_s extiCallbackRec_t;
typedef void extiHandlerCallback(extiCallbackRec_t *self);

struct extiCallbackRec_s {
	extiHandlerCallback *fn;
};

void EXTIInit(void);
void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn);
void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, EXTITrigger_TypeDef trigger);
void EXTIEnable(IO_t io, bool enable);
void EXTIRelease(IO_t io);

#endif	// __EXTI_H
