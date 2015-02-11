#ifndef DS_H_
#define DS_H_
#include <CANTalon.h>
#include <Encoder.h>
#include "Log.h"

class DS
{
public:
	static DS* getInstance();

	void process();

	enum Direction {
		FORWARD = 1,
		STILL = 0,
		BACKWARD = -1
	};

	Direction frontLeftMotorDirection();
	Direction frontRightMotorDirection();
	Direction rearLeftMotorDirection();
	Direction rearRightMotorDirection();

	Direction lifterMotorDirection();
	Direction armFlapMotorDirection();
	Direction portRakeMotorDirection();
	Direction starboardRakeMotorDirection();

private:
	DS();
	static DS* INSTANCE;

	Log* log;

	Encoder* encoder;
	Encoder* od_xaxis_encoder;
	Encoder* od_yaxis_encoder;

	Joystick* joystick_one;
	Joystick* joystick_two;
	Joystick* joystick_three;
};

#endif // DS_H_
