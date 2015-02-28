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

I2CGyro::I2CGyro()
{
	timer = new Timer();
	channel = new I2C(I2C::kOnboard, RobotPorts::GYRO_ADDRESS);

	last_angle = 0.0;
	current_angle = 0.0;

	rate = 0.0;
	timer->Start();
}

void I2CGyro::process()
{

}

double I2CGyro::PIDGet()
{
	return 0.0;
}

I2CGyro* I2CGyro::getInstance()
{
	if (INSTANCE == NULL) {
		INSTANCE = new I2CGyro();
	}
	return INSTANCE;
}
