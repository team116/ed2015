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
#include <WPILib.h>

I2CGyro* I2CGyro::INSTANCE = NULL;

const float I2CGyro::READ_DELAY = 0.05f;
const float I2CGyro::CONFIG_DELAY = 5.0f;
const unsigned int I2CGyro::BUF_SIZE = 6;

I2CGyro::I2CGyro()
{
	log = Log::getInstance();
	log->write(Log::DEBUG_LEVEL, "%s\tConstructing I2CGyro.cpp\n", Utils::getCurrentTime());

	timer = new Timer();
	channel = new I2C(I2C::kOnboard, RobotPorts::GYRO_ADDRESS);

	last_angle = 0.0;
	current_angle = 0.0;
	offset = 0.0;

	channel->Write(RobotPorts::GYRO_REG_POWER, 0x80);
	next_step = RANGE_AND_BANDWIDTH;

	timer->Start();
	timer->Reset();
}

void I2CGyro::process()
{
	log->write(Log::TRACE_LEVEL, "Last Angle: %f\nCurrent Angle: %f\nOffset: %f\n\n", last_angle, current_angle, offset);
	if (next_step == DONE && timer->Get() > READ_DELAY) {
		last_angle = current_angle;

		unsigned char buff[BUF_SIZE];
		channel->Read(RobotPorts::GYRO_REG_MXSB, BUF_SIZE, buff);
		current_angle = (float)(-1 * ((((short)buff[4]) << 8) | buff[5]));

		timer->Reset();
	}
	else if (timer->Get() > CONFIG_DELAY) {
		switch (next_step) {
		case RANGE_AND_BANDWIDTH:
			channel->Write(RobotPorts::GYRO_REG_DLPF_FS, 0x1B);
			break;
		case SAMPLE_RATIO:
			channel->Write(RobotPorts::GYRO_REG_SAMPLE_RATIO, 0x0A);
			break;
		case PLL:
			channel->Write(RobotPorts::GYRO_REG_PLL, 0x00);
			break;
		default:
			// we got a problem
			break;
		}
		timer->Reset();
	}
}

void I2CGyro::reset() {
	offset = current_angle;
}

double I2CGyro::PIDGet()
{
	// return getAngle();
	return 0.0;
}

float I2CGyro::getRate()
{
	//return (last_angle - current_angle) / READ_DELAY;
	return 0.0;
}

float I2CGyro::getAngle()
{
	//return current_angle - offset;
	return 0.0;
}

I2CGyro* I2CGyro::getInstance()
{
	if (INSTANCE == NULL) {
		INSTANCE = new I2CGyro();
	}
	return INSTANCE;
}
