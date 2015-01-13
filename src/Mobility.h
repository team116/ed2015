#ifndef MOBILITY_H_
#define MOBILITY_H_

#include "WPILib.h"
#include "Gyro.h"
#include "TalonSRX.h"

class Mobility
{
public:
	static Mobility* getInstance();
	void process();
	void setDirection(float x, float y);
	void setRotation(float rotation);

private:
	Mobility();
	static Mobility* INSTANCE;
	TalonSRX* front_left_motor;
	TalonSRX* front_right_motor;
	TalonSRX* rear_left_motor;
	TalonSRX* rear_right_motor;
	RobotDrive* robot_drive;

	float x_direction;
	float y_direction;
	float rotation;
};

#endif // MOBILITY_H_
