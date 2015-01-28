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
	if (joystick_one->GetRawButton(JoystickPorts::CLOSE_FLAPS)){
		flaps->Set(0.5);
	}
	if (joystick_one ->GetRawButton(JoystickPorts::OPEN_FLAPS)){
		flaps->Set(-0.5);
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
