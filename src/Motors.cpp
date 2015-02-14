/*
 * Motors.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: Lenovo
 */

#include "Motors.h"
#include "Ports.h"

Motors* Motors::INSTANCE = NULL;

Motors::Motors() {
	// TODO Auto-generated constructor stub

	ds = DS::getInstance();

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

}
void Motors::process() {
	//mobility
	//front left wheel
	DS::Direction direction = ds->frontLeftMotorDirection();
	if (direction == DS::FORWARD) {
		front_left_wheel->Set(0.5);
	}
	else if (direction == DS::STILL) {
		front_left_wheel->Set(0.0);
	}
	else if (direction == DS::BACKWARD) {
		front_left_wheel->Set(-0.5);
	}

	//front right wheel
	direction = ds->frontRightMotorDirection();
	if (direction == DS::FORWARD) {
		front_right_wheel->Set(0.5);
	}
	else if (direction == DS::STILL) {
		front_right_wheel->Set(0.0);
	}
	else if (direction == DS::BACKWARD) {
		front_right_wheel->Set(-0.5);
	}

	//rear left wheel
	direction = ds->rearLeftMotorDirection();
	if (direction == DS::FORWARD) {
		rear_left_wheel->Set(0.5);
	}
	else if (direction == DS::STILL) {
		rear_left_wheel->Set(0.0);
	}
	else if (direction == DS::BACKWARD) {
		rear_left_wheel->Set(-0.5);
	}

	//rear right wheel
	direction = ds->rearRightMotorDirection();
	if (direction == DS::FORWARD) {
		rear_right_wheel->Set(0.5);
	}
	else if (direction == DS::STILL) {
		rear_right_wheel->Set(0.0);
	}
	else if (direction == DS::BACKWARD) {
		rear_right_wheel->Set(-0.5);
	}

	//flaps
	direction = ds->armFlapMotorDirection();
	if (direction == DS::FORWARD) {
		flaps->Set(0.5);
		//closing
	}
	else if (direction == DS::BACKWARD) {
		flaps->Set(-0.5);
		//opening
	}
	else if (direction == DS::STILL) {
		flaps->Set(0.0);
	}

	//port rake
	direction = ds->portRakeMotorDirection();
	if (direction == DS::FORWARD) {
		rake_port->Set(0.5);
	}
	else if (direction == DS::BACKWARD) {
		rake_port->Set(-0.5);
	}
	else if (direction == DS::STILL) {
		rake_port->Set(0.0);
	}

	//port starboard
	direction = ds->starboardRakeMotorDirection();
	if (direction == DS::FORWARD) {
		rake_starboard->Set(0.5);
	}
	else if (direction == DS::BACKWARD) {
		rake_starboard->Set(-0.5);
	}
	else if (direction == DS::STILL) {
		rake_starboard->Set(0.0);
	}

	//lifter
	direction = ds->lifterMotorDirection();
	if (direction == DS::FORWARD) {
		moveLifter(0.5);
	}
	else if (direction == DS::BACKWARD) {
		moveLifter(-0.5);
	}
	else if (direction == DS::STILL) {
		moveLifter(0.0);
	}

}

void Motors::moveLifter(float volts) {
	lifter_one->Set(volts);
	lifter_two->Set(volts);
}

Motors* Motors::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new Motors();
	}
	return INSTANCE;
}
