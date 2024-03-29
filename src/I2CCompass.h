/*
 * I2CCompass.h
 *
 *  Created on: Feb 19, 2015
 *      Author: calcifer
 */

#ifndef SRC_I2CCOMPASS_H_
#define SRC_I2CCOMPASS_H_

#include <PIDSource.h>
#include <Timer.h>
#include <I2C.h>
#include "Log.h"

class I2CCompass : PIDSource {
public:
	static I2CCompass* getInstance();

	void process();

	virtual double PIDGet();
	float getYaw();

private:
	I2CCompass();
	static I2CCompass* INSTANCE;

	Log* log;

	Timer* timer;
	I2C* channel;

	static const unsigned int BUF_SIZE;

	float yaw;
};

#endif /* SRC_I2CCOMPASS_H_ */
