/*
 * SensorStick.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: Lenovo
 */

#include <cmath>
#include <SensorStick.h>
#include <Timer.h>

SensorStick* SensorStick::INSTANCE = NULL;
const int SensorStick::ACCEL_ADDRESS = 0x53;
const int SensorStick::ACCEL_PWR_CTRL_PORT = 0x2D;
const int SensorStick::ACCEL_DATA_FORMAT_PORT = 0x31;
const int SensorStick::ACCEL_XLSB_PORT = 0x32;
// 2 bytes per axis
const int SensorStick::ACCEL_BUFFER_SIZE = 6;

const int SensorStick::COMPASS_ADDRESS = 0x1E;
const int SensorStick::COMPASS_MEASURE_MODE_PORT = 0x02;
const int SensorStick::COMPASS_XMSB_PORT = 0x03;
// used to set the measurement mode to 'continuous'
#define COMPASS_MEASURE_MODE_CONT 0x00
const int SensorStick::COMPASS_BUFFER_SIZE = 6;

#define PI 3.1415926

SensorStick::SensorStick()
{
	// ADXL345 initialization (accelerometer)
	accel_y_axis = 0;
	accel_x_axis = 0;
	accel_z_axis = 0;
	accel_channel = new I2C(I2C::kOnboard, ACCEL_ADDRESS);
	accel_timer = new Timer();
	accel_channel->Write(ACCEL_PWR_CTRL_PORT, 0);
	accel_channel->Write(ACCEL_PWR_CTRL_PORT, 16);
	accel_channel->Write(ACCEL_PWR_CTRL_PORT, 8);
	accel_timer->Start();

	// HMC5843 initialization (compass)
	compass_yaw = 0;
	compass_channel = new I2C(I2C::kOnboard, COMPASS_ADDRESS);
	compass_timer = new Timer();
	compass_channel->Write(COMPASS_MEASURE_MODE_PORT, COMPASS_MEASURE_MODE_CONT);
}

void SensorStick::process()
{
	accelProcess();
	compassProcess();
	gyroProcess();
}

void SensorStick::accelProcess()
{
	// 50 second delay to prevent channel clogging
	if (accel_timer->Get() > 0.05) {
		unsigned char buff[ACCEL_BUFFER_SIZE];
		accel_channel->Read(ACCEL_XLSB_PORT, ACCEL_BUFFER_SIZE, buff);
		accel_x_axis = (((int)buff[1]) << 8) | buff[0];
		accel_y_axis = (((int)buff[3]) << 8) | buff[2];
		accel_z_axis = (((int)buff[5]) << 8) | buff[4];

		accel_timer->Reset();
	}
}

void SensorStick::compassProcess()
{
	if (compass_timer->Get() > 0.05) {
		unsigned char buff[COMPASS_BUFFER_SIZE];
		short compass_data[3];
		compass_channel->Read(COMPASS_XMSB_PORT, COMPASS_BUFFER_SIZE, buff);

		for (int i = 0; i < 3; ++i) {
			compass_data[i] = (short)buff[2 * i + 1] + (((short)buff[2 * i]) << 8);
		}
		compass_yaw = atan2(compass_data[2], compass_data[0]) * 180.0 / PI;

		compass_timer->Reset();
	}
}

void SensorStick::gyroProcess()
{

}

int SensorStick::getAccelX()
{
	return accel_x_axis;
}

int SensorStick::getAccelY()
{
	return accel_y_axis;
}

int SensorStick::getAccelZ()
{
	return accel_z_axis;
}

float SensorStick::getCompassYaw()
{
	return compass_yaw;
}

SensorStick* SensorStick::getInstance()
{
	if (INSTANCE == NULL) {
		INSTANCE = new SensorStick();
	}
	return INSTANCE;
}
