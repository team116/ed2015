#ifndef DS_H_
#define DS_H_
#include <CANTalon.h>
#include <Encoder.h>
#include <Joystick.h>
#include "Log.h"
#include "PortsByMotor.h"

class DS
{
public:
	static DS* getInstance();

	void process();

	/*enum Direction {
		FORWARD = 1,
		STILL = 0,
		BACKWARD = -1
	};*/

	PortsByMotor* front_left_motor;
	PortsByMotor* front_right_motor;
	PortsByMotor* rear_left_motor;
	PortsByMotor* rear_right_motor;

	PortsByMotor* lift_motor;
	PortsByMotor* flap_motor;
	PortsByMotor* port_rake_motor;
	PortsByMotor* starboard_rake_motor;
	PortsByMotor* left_tote_motor;
	PortsByMotor* right_tote_motor;

	//Direction getDirection(PortsByMotor* motor);	//moved to PortsByMotor
	/*Direction frontLeftMotorDirection();
	Direction frontRightMotorDirection();
	Direction rearLeftMotorDirection();
	Direction rearRightMotorDirection();

	Direction lifterMotorDirection();
	Direction armFlapMotorDirection();	//note: forward = closing, backward = opening
	Direction portRakeMotorDirection();
	Direction starboardRakeMotorDirection();
	Direction leftToteWheel();
	Direction rightToteWheel();*/

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
