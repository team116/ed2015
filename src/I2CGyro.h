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

class I2CGyro : PIDSource {
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

	static const unsigned int BUF_SIZE;

	I2C* channel;
	// used to prevent heavy bus traffic
	Timer* timer;

	static const float READ_DELAY;

	float last_angle;
	float current_angle;
	float offset;

	float rate;

};

#endif /* SRC_I2CGYRO_H_ */
