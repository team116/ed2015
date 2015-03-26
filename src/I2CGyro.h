/*
 * I2CGyro.h
 *
 *  Created on: Feb 19, 2015
 *      Author: calcifer
 */

#ifndef SRC_I2CGYRO_H_
#define SRC_I2CGYRO_H_

#include <Gyro.h>
#include <PIDSource.h>
#include <I2C.h>
#include <Timer.h>
#include <WPILib.h>
#include "Log.h"

class I2CGyro : public PIDSource {
public:
	static I2CGyro* getInstance();

	void process();

	virtual double PIDGet();
	float getAngle();
	float getRate();
	void reset();

private:
	I2CGyro();
	static I2CGyro* INSTANCE;

	enum InitStep {
		POWER_UP = 0,
		RANGE_AND_BANDWIDTH = 1,
		SAMPLE_RATIO = 2,
		PLL = 3,
		DONE = 4
	};
	InitStep next_step;

	I2C* channel;
	// used to prevent heavy bus traffic
	Timer* timer;
	Log* log;

	static const float READ_DELAY;
	static const float CONFIG_DELAY;
	static const unsigned int BUF_SIZE;

	float last_angle;
	float current_angle;
	float offset = 0.0;

};

#endif /* SRC_I2CGYRO_H_ */
