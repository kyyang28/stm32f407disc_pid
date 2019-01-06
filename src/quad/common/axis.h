#ifndef __AXIS_H
#define __AXIS_H

#define XYZ_AXIS_COUNT						3

#define FLIGHT_DYNAMICS_INDEX_COUNT			3

/* See http://en.wikipedia.org/wiki/Flight_dynamics */
typedef enum {
	FD_ROLL = 0,		// 0
	FD_PITCH,			// 1
	FD_YAW				// 2
}flight_dynamics_index_t;

typedef enum {
	X = 0,				// X = 0
	Y,					// Y = 1
	Z					// Z = 2
}axis_e;

#endif	// __AXIS_H
