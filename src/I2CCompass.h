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

class I2CCompass : PIDSource {
public:
	static I2CCompass* getInstance();

	void process();

	virtual double PIDGet();

private:
	I2CCompass();
	static I2CCompass* INSTANCE;

	Timer* timer;
	I2C* channel;

	static const unsigned int BUF_SIZE;

	float yaw;
};

#endif /* SRC_I2CCOMPASS_H_ */
