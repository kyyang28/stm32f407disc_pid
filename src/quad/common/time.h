#ifndef __TIME_H
#define __TIME_H

#include <stdint.h>

/* time difference, 32 bits always sufficient */
typedef int32_t timeDelta_t;

/* millisecond time */
typedef uint32_t timeMs_t;

/* microsecond time */
typedef uint32_t timeUs_t;

#define TIMEUS_MAX  UINT32_MAX

#endif	// __TIME_H
