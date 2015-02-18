/*
 * SensorStick.h
 *
 *  Created on: Feb 14, 2015
 *      Author: Lenovo
 */

#ifndef SRC_SENSORSTICK_H_
#define SRC_SENSORSTICK_H_

#include <I2C.h>
#include <Timer.h>

class SensorStick
{
public:
	static SensorStick* getInstance();

	void process();

	int getAccelX();
	int getAccelY();
	int getAccelZ();

	float getCompassYaw();

private:
	SensorStick();
	static SensorStick* INSTANCE;

	void accelProcess();
	void compassProcess();
	void gyroProcess();

	I2C* accel_channel;
	Timer* accel_timer;
	short accel_y_axis;
	short accel_x_axis;
	short accel_z_axis;

	// slave address for the ADXL345
	static const int ACCEL_ADDRESS;
	// ADXL345 configuration port
	static const int ACCEL_PWR_CTRL_PORT;
	static const int ACCEL_DATA_FORMAT_PORT;
	static const int ACCEL_XLSB_PORT;
	static const int ACCEL_BUFFER_SIZE;

	I2C* compass_channel;
	Timer* compass_timer;
	float compass_yaw;

	// slave address for the HMC5843
	static const int COMPASS_ADDRESS;
	static const int COMPASS_MEASURE_MODE_PORT;
	static const int COMPASS_BUFFER_SIZE;
	static const int COMPASS_XMSB_PORT;


};

#endif /* SRC_SENSORSTICK_H_ */
