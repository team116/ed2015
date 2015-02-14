#ifndef DS_H_
#define DS_H_
#include <CANTalon.h>
#include <Encoder.h>
#include <Joystick.h>
#include "Log.h"

class DS {
public:
	static DS* getInstance();

	void process();

	enum Direction {
		FORWARD, STILL, BACKWARD
	};

	Direction frontLeftMotorDirection();
	Direction frontRightMotorDirection();
	Direction rearLeftMotorDirection();
	Direction rearRightMotorDirection();

	Direction lifterMotorDirection();
	Direction armFlapMotorDirection();	//note: forward = closing, backward = opening
	Direction portRakeMotorDirection();
	Direction starboardRakeMotorDirection();
	Direction leftToteWheelDirection();
	Direction rightToteWheelDirection();

private:
	DS();
	static DS* INSTANCE;

	Direction getDirection(Joystick* joystick, int port_1, int port_2);
	Log* log;

	Encoder* encoder;
	Encoder* od_xaxis_encoder;
	Encoder* od_yaxis_encoder;

	Joystick* joystick_one;
	Joystick* joystick_two;
	Joystick* joystick_three;
};

#endif // DS_H_
