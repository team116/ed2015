#include <WPILib.h>
#include "Ports.h"
#include "DS.h"
#include "Log.h"
#include <CameraServer.h>
DS* DS::INSTANCE = NULL;

DS::DS()
{
	front_left_wheel = new CANTalon(RobotPorts::FRONT_LEFT_MOTOR);
	front_right_wheel = new CANTalon(RobotPorts::FRONT_RIGHT_MOTOR);
	rear_left_wheel = new CANTalon(RobotPorts::REAR_LEFT_MOTOR);
	rear_right_wheel = new CANTalon(RobotPorts::REAR_RIGHT_MOTOR);
	left_grabber_wheel = new CANTalon(RobotPorts::LEFT_WHEEL);
	right_grabber_wheel = new CANTalon(RobotPorts::RIGHT_WHEEL);
	lifter_one = new CANTalon(RobotPorts::LIFTER_ONE);
	lifter_two = new CANTalon(RobotPorts::LIFTER_TWO);
	rake_port = new CANTalon(RobotPorts::RAKE_PORT_MOTOR);
	rake_starboard = new CANTalon(RobotPorts::RAKE_STARBOARD_MOTOR);
	flaps = new CANTalon(RobotPorts::CLOSE_FLAPS_MOTOR);
	joystick_one = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	joystick_two = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	joystick_three = Joystick::GetStickForPort(DSPorts::BUTTONS_JOYSTICK);
}

void DS::process()
{
	// a !XOR would be nice here... would get rid of the else at the end, but I think this is more clear
	// could do !(button1 ^ button2)
	if (joystick_one->GetRawButton(JoystickPorts::CLOSE_FLAPS) &&
		joystick_one->GetRawButton(JoystickPorts::OPEN_FLAPS)) {
		flaps->Set(0.0);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::CLOSE_FLAPS)){
		flaps->Set(0.5);
	}
	else if (joystick_one ->GetRawButton(JoystickPorts::OPEN_FLAPS)){
		flaps->Set(-0.5);
	}
	else {
		flaps->Set(0.0);
	}

	if (joystick_one->GetRawButton(JoystickPorts::LIFT_STARBOARD_RAKE) &&
		joystick_one->GetRawButton(JoystickPorts::LOWER_STARBOARD_RAKE)) {
		rake_starboard->Set(0.0);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::LIFT_STARBOARD_RAKE)) {
		rake_starboard->Set(0.5);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::LOWER_STARBOARD_RAKE)) {
		rake_starboard->Set(-0.5);
	}
	else {
		rake_starboard->Set(0.0);
	}

	if (joystick_one->GetRawButton(JoystickPorts::LIFT_PORT_RAKE) &&
		joystick_one->GetRawButton(JoystickPorts::LOWER_PORT_RAKE)) {
		// both up and down buttons pressed, don't move
		rake_port->Set(0.0);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::LIFT_PORT_RAKE)) {
		rake_port->Set(0.5);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::LOWER_PORT_RAKE)) {
		rake_port->Set(-0.5);
	}
	else {
		rake_port->Set(0.0);
	}
}

DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
