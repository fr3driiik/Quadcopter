#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino.h"
#include "Wire.h"
#include "Utils.h"
#include "Config.h"

#define X_AXIS	0
#define Y_AXIS	1
#define Z_AXIS	2

#define OUTPUT_ERRORS

namespace Sensors {
	extern float gyro[3];
	extern float accel[3]; 
	extern float magnetom[3]; 

	void initGyro();
	void initAcc();
	void initMag();
	void readGyro();
	void readAcc();
	void readMag();
}
#endif
