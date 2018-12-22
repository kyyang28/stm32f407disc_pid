#ifndef __BAROMETER_H
#define __BAROMETER_H

typedef enum {
	BARO_DEFAULT = 0,
	BARO_NONE = 1,
	BARO_BMP085 = 2,
	BARO_MS5611 = 3,
	BARO_BMP280 = 4
}baroSensor_e;

#endif	// __BAROMETER_H
