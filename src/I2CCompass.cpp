/*
 * I2CCompass.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: calcifer
 */

#include "I2CCompass.h"
#include "Ports.h"
#include <cmath>
#include <I2C.h>
#include <Timer.h>

#define PI 3.1415926
// the number of bytes in one coordinate sent by the compass, unsigned
#define DATA_SIZE 2U
I2CCompass* I2CCompass::INSTANCE = NULL;
const unsigned int I2CCompass::BUF_SIZE = 6;

I2CCompass::I2CCompass()
{
	timer = new Timer();
	channel = new I2C(I2C::kOnboard, RobotPorts::COMPASS_ADDRESS);

	yaw = 0.0;
	timer->Start();
}

void I2CCompass::process()
{
	if (timer->Get() > 0.05) {
		unsigned char buff[BUF_SIZE];
		short compass_data[BUF_SIZE / DATA_SIZE];
		channel->Read(RobotPorts::COMPASS_REG_XMSB, BUF_SIZE, buff);

		for (unsigned int i = 0; i < BUF_SIZE / DATA_SIZE; ++i) {
			compass_data[i] = (short)buff[2 * i + 1] + (((short)buff[2 * i]) << 8);
		}
		yaw = atan2(compass_data[2], compass_data[0]) * 180.0 / PI;

		timer->Reset();
	}
}

double I2CCompass::PIDGet()
{
	return 0.0;
}

float I2CCompass::getYaw()
{
	return yaw;
}

I2CCompass* I2CCompass::getInstance()
{
	if (INSTANCE == NULL) {
		INSTANCE = new I2CCompass();
	}
	return INSTANCE;
}
