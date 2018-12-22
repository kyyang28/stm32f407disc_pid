#ifndef __IODEF_H
#define __IODEF_H

#include "utils.h"
#include "IOTypes.h"
#include "target.h"				/* TARGET must define used pins */
#include "io_def_generated.h"	/* Include template-generated macros for IO pins */


/* tag of NONE must be false */
#define DEFIO_TAG__NONE						0

/* return ioTag_t for given pinID */
#define DEFIO_TAG(pinID)					CONCAT(DEFIO_TAG__, pinID)


#define DEFIO_REC(pinID)					CONCAT(DEFIO_REC__, pinID)
#define DEFIO_REC__NONE						NULL

#define DEFIO_IO(pinID)						(IO_t)DEFIO_REC(pinID)

/* ioTag_t accessor macros */
/* DEFIO_TAG__<port> format(8 bits): xxxxyyyy, where xxxx is gpioID, yyyy is pinID */
#define DEFIO_TAG_MAKE(gpioID, pin)			((ioTag_t)((((gpioID) + 1) << 4) | (pin)))
#define DEFIO_TAG_GPIOID(tag)				(((tag) >> 4) - 1)
#define DEFIO_TAG_PIN(tag)					((tag) & 0x0F)

#endif	// __IODEF_H
