#ifndef MOBILITY_H_
#define MOBILITY_H_

#include <WPILib.h>
#include <Gyro.h>
#include <AnalogInput.h>
#include <CANTalon.h>
#include "Log.h"

class Mobility
{
public:
	static Mobility* getInstance();
	void process();
	void setDirection(float x, float y);
	void setRotation(float rotation);
	float getUltrasonicDistance();
	void runTalon(int talon, float speed);

private:
	Mobility();
	static Mobility* INSTANCE;
	CANTalon* front_left_motor;
	CANTalon* front_right_motor;
	CANTalon* rear_left_motor;
	CANTalon* rear_right_motor;
	RobotDrive* robot_drive;
	AnalogInput* ultrasonic;

	Log* log;

	float x_direction;
	float y_direction;
	float rotation;
};

#endif // MOBILITY_H_
