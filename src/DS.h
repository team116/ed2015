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

	CANTalon* front_left_wheel;
	CANTalon* front_right_wheel;
	CANTalon* rear_left_wheel;
	CANTalon* rear_right_wheel;
	CANTalon* left_grabber_wheel;
	CANTalon* right_grabber_wheel;
	CANTalon* lifter_one;
	CANTalon* lifter_two;
	CANTalon* rake_port;
	CANTalon* rake_starboard;
	CANTalon* flaps;
	Joystick* joystick_one;
	Joystick* joystick_two;
	Joystick* joystick_three;
};

#endif // DS_H_
