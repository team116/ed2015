/*
 * Sensors.h
 *
 *  Created on: Feb 12, 2015
 *      Author: Lenovo
 */

#ifndef SRC_SENSORS_H_
#define SRC_SENSORS_H_

#include <Encoder.h>
#include <AnalogInput.h>
#include "Log.h"

class Sensors {
public:
	static Sensors* getInstance();

	void process();

private:
	Sensors();
	static Sensors* INSTANCE;

	AnalogInput* ultrasonic;

	Encoder* od_xaxis_encoder;
	Encoder* od_yaxis_encoder;

	Log* log;
};

#endif /* SRC_SENSORS_H_ */
