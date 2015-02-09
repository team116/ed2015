#include <WPILib.h>
#include <Encoder.h>
#include "Ports.h"
#include "DS.h"
#include "Log.h"
#include <CameraServer.h>
DS* DS::INSTANCE = NULL;

DS::DS()
{
	encoder = new Encoder(RobotPorts::ENCODER_A, RobotPorts::ENCODER_B);

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

	log = Log::getInstance();
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



	//wheels
	if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_LEFT_MOTOR) &&
		joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_FRONT_LEFT_MOTOR)) {
		front_left_wheel->Set(0.0);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_LEFT_MOTOR)) {
		front_left_wheel->Set(0.5);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_FRONT_LEFT_MOTOR)) {
		front_left_wheel->Set(-0.5);
	}
	else {
		front_left_wheel->Set(0.0);
	}

	if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_RIGHT_MOTOR) &&
		joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR)) {
		front_right_wheel->Set(0.0);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_RIGHT_MOTOR)) {
		front_right_wheel->Set(0.5);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR)) {
		front_right_wheel->Set(-0.5);
	}
	else {
		front_right_wheel->Set(0.0);
	}

	if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_LEFT_MOTOR) &&
		joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_REAR_LEFT_MOTOR)) {
		rear_left_wheel->Set(0.0);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_LEFT_MOTOR)) {
		rear_left_wheel->Set(0.5);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_REAR_LEFT_MOTOR)) {
		rear_left_wheel->Set(-0.5);
	}
	else {
		rear_left_wheel->Set(0.0);
	}

	if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_RIGHT_MOTOR) &&
		joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_REAR_RIGHT_MOTOR)) {
		rear_right_wheel->Set(0.0);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_RIGHT_MOTOR)) {
		rear_right_wheel->Set(0.5);
	}
	else if (joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_REAR_RIGHT_MOTOR)) {
		rear_right_wheel->Set(-0.5);
	}
	else {
		rear_right_wheel->Set(0.0);
	}

	//joystick two
	//rakes
	if (joystick_two->GetRawButton(JoystickPorts::LIFT_STARBOARD_RAKE) &&
			joystick_two->GetRawButton(JoystickPorts::LOWER_STARBOARD_RAKE)) {
			rake_starboard->Set(0.0);
	}
	else if (joystick_two->GetRawButton(JoystickPorts::LIFT_STARBOARD_RAKE)) {
		rake_starboard->Set(0.5);
	}
	else if (joystick_two->GetRawButton(JoystickPorts::LOWER_STARBOARD_RAKE)) {
		rake_starboard->Set(-0.5);
	}
	else {
		rake_starboard->Set(0.0);
	}

	if (joystick_two->GetRawButton(JoystickPorts::LIFT_PORT_RAKE) &&
		joystick_two->GetRawButton(JoystickPorts::LOWER_PORT_RAKE)) {
		// both up and down buttons pressed, don't move
		rake_port->Set(0.0);
	}
	else if (joystick_two->GetRawButton(JoystickPorts::LIFT_PORT_RAKE)) {
		rake_port->Set(0.5);
	}
	else if (joystick_two->GetRawButton(JoystickPorts::LOWER_PORT_RAKE)) {
		rake_port->Set(-0.5);
	}
	else {
		rake_port->Set(0.0);
	}
	//manipulator grabber wheels
	if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_LEFT_WHEEL) &&
		joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_LEFT_WHEEL)) {
		left_grabber_wheel->Set(0.0);
	}
	else if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_LEFT_WHEEL)) {
		left_grabber_wheel->Set(0.5);
	}
	else if (joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_LEFT_WHEEL)) {
		left_grabber_wheel->Set(-0.5);
	}
	else {
		left_grabber_wheel->Set(0.0);
	}

	if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_RIGHT_WHEEL) &&
		joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_RIGHT_WHEEL)) {
		right_grabber_wheel->Set(0.0);
	}
	else if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_RIGHT_WHEEL)) {
		right_grabber_wheel->Set(0.5);
	}
	else if (joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_RIGHT_WHEEL)) {
		right_grabber_wheel->Set(-0.5);
	}
	else {
		right_grabber_wheel->Set(0.0);
	}
	//lifters for manipulator
	if  (joystick_two->GetRawButton(JoystickPorts::GOING_UP_LIFTER) &&
		joystick_two->GetRawButton(JoystickPorts::GOING_DOWN_LIFTER)) {
			lifter_one->Set(0.0);
			lifter_two->Set(0.0);
		}
	else if (joystick_two->GetRawButton(JoystickPorts::GOING_UP_LIFTER)) {
			lifter_one->Set(0.5);
			lifter_two->Set(0.5);
	}
	else if (joystick_two->GetRawButton(JoystickPorts::GOING_DOWN_LIFTER)) {
		lifter_one->Set(-0.5);
		lifter_two->Set(-0.5);
	}
	else {
		lifter_one->Set(0.0);
		lifter_two->Set(0.0);
	}

	/*
	log->write(Log::TRACE_LEVEL, "-----------Talon Encoder-------------\n");
	log->write(Log::INFO_LEVEL, "Distance: %i\n", rear_left_wheel->GetEncPosition());
	log->write(Log::INFO_LEVEL, "Velocity: %i\n", rear_left_wheel->GetEncVel());

	log->write(Log::TRACE_LEVEL, "----------Digital Encoder------------\n");
	*/
	log->write(Log::INFO_LEVEL, "Get: %i\n", encoder->Get());
	log->write(Log::INFO_LEVEL, "Get Raw: %i\n", encoder->GetRaw());
	log->write(Log::INFO_LEVEL, "Rate: %f\n", encoder->GetRate());
	log->write(Log::INFO_LEVEL, "Get Encoding Scale: %i\n", encoder->GetEncodingScale());
}

DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
