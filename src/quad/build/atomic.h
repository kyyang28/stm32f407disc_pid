
#pragma once

#include "core_cmFunc.h"

#if 1
// set BASEPRI_MAX function, with global memory barrier, returns true
static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
    __set_BASEPRI_MAX(prio);
    return 1;
}

// cleanup BASEPRI restore function, with global memory barrier
static inline void __basepriRestoreMem(uint8_t *val)
{
    __set_BASEPRI(*val);
}
#endif

// Run block with elevated BASEPRI (using BASEPRI_MAX), restoring BASEPRI on exit. All exit paths are handled
// Full memory barrier is placed at start and exit of block
//#define ATOMIC_BLOCK(prio) for ( uint8_t __basepri_save __attribute__((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI(), __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )
