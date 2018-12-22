#ifndef __UTILS_H
#define __UTILS_H

#include <stddef.h>
#include <stdint.h>

#define ARRAYLEN(x)					(sizeof(x) / sizeof((x)[0]))
#define ARRAYEND(x)					(&(x)[ARRAYLEN(x)])

#define CONCAT_HELPER(x, y)			x ## y
#define CONCAT(x, y)				CONCAT_HELPER(x, y)

#define BIT(x)						(1 << (x))

#define BX_(x)						((x) - (((x) >> 1) & 0x77777777) - (((x) >> 2) & 0x33333333) - (((x) >> 3) & 0x11111111))
#define BITCOUNT(x)					(((BX_(x) + (BX_(x) >> 4)) & 0x0F0F0F0F) % 255)


/*
 * https://groups.google.com/forum/?hl=en#!msg/comp.lang.c/attFnqwhvGk/sGBKXvIkY3AJ
 * Return (v ? floor(log2(v)) : 0) when 0 <= v < 1<<[8, 16, 32, 64].
 * Inefficient algorithm, intended for compile-time constants.
 */
 #define LOG2_8BIT(v)				(8 - 90 / (((v) / 4 + 14) | 1) - 2 / ((v) / 2 + 1))
 #define LOG2_16BIT(v)				(8 * ((v) > 255/* 2^8 */) + LOG2_8BIT((v) >> 8 * ((v) > 255)))
 #define LOG2_32BIT(v)				(16 * ((v) > 65535L/* 2^16 */) + LOG2_16BIT((v) * 1L >> 16 * ((v) > 65535L)))

#if !defined(UNUSED)
#define UNUSED(x)					(void)(x)
#endif

#if 1
// ISO C version, but no type checking
#define container_of(ptr, type, member) \
                      ((type *) ((char *)(ptr) - offsetof(type, member)))
#else
// non ISO variant from linux kernel; checks ptr type, but triggers 'ISO C forbids braced-groups within expressions [-Wpedantic]'
//  __extension__ is here to disable this warning
#define container_of(ptr, type, member)  ( __extension__ ({     \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );}))
#endif

#endif	// __UTILS_H
