#ifndef MOBILITY_H_
#define MOBILITY_H_

#include <WPILib.h>
#include <Gyro.h>
#include <AnalogInput.h>
#include <CANTalon.h>
#include "Log.h"
#include <BuiltInAccelerometer.h>

class Mobility
{
public:
	static Mobility* getInstance();
	void process();
	void setDirection(float x, float y);
	void setRotation(float rotation);
	float getUltrasonicDistance();
	void runTalon(int talon, float speed);
	void toggleFieldCentric();

private:
	Mobility();
	static Mobility* INSTANCE;
	CANTalon* front_left_motor;
	CANTalon* front_right_motor;
	CANTalon* rear_left_motor;
	CANTalon* rear_right_motor;
	RobotDrive* robot_drive;
	AnalogInput* ultrasonic;
	BuiltInAccelerometer* accel;
	Gyro* gyro;

	Log* log;

	bool field_centric;
	float x_direction;
	float y_direction;
	float rotation;
};

#endif // MOBILITY_H_
