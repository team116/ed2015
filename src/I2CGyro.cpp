/*
 * I2CGyro.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: calcifer
 */

#include "I2CGyro.h"
#include <I2C.h>
#include <Timer.h>
#include "Ports.h"

I2CGyro* I2CGyro::INSTANCE = NULL;

#define DATA_SIZE 2U

const unsigned int I2CGyro::BUF_SIZE = 6;

I2CGyro::I2CGyro()
{
	timer = new Timer();
	channel = new I2C(I2C::kOnboard, RobotPorts::GYRO_ADDRESS);

	last_angle = 0.0;
	current_angle = 0.0;
	offset = 0.0;

	rate = 0.0;
	timer->Start();
}

void I2CGyro::process()
{
	if(timer->Get() > 0.05) {
		unsigned char buff[BUF_SIZE];
		short gyro_data[BUF_SIZE / DATA_SIZE];
		channel->Read(RobotPorts::GYRO_REG_MXSB, BUF_SIZE, buff);

		for (unsigned int i = 0; i < BUF_SIZE / DATA_SIZE; ++i) {
			gyro_data[i] = (short)buff[2 * i + 1] + (((short)buff[2 * i]) << 8);
		}
	}
}

void I2CGyro::reset() {
	offset = current_angle;
}

double I2CGyro::PIDGet()
{
	return 0.0;
}

float I2CGyro::getRate()
{
	return rate;
}

float I2CGyro::getAngle()
{
	return current_angle - offset;
}

I2CGyro* I2CGyro::getInstance()
{
	if (INSTANCE == NULL) {
		INSTANCE = new I2CGyro();
	}
	return INSTANCE;
}
