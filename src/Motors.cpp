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
	log = Log::getInstance();

}
void Motors::process() {
	//mobility
	//front left wheel
	setMotorDirection(ds->front_left_motor, front_left_wheel);
	setMotorDirection(ds->front_right_motor, front_right_wheel);
	setMotorDirection(ds->rear_left_motor, rear_left_wheel);
	setMotorDirection(ds->rear_right_motor, rear_right_wheel);
	setMotorDirection(ds->flap_motor, flaps);
	setMotorDirection(ds->port_rake_motor, rake_port);
	setMotorDirection(ds->starboard_rake_motor, rake_starboard);

	moveLifter();


	log->write(Log::INFO_LEVEL, "Get: Front Left Wheel Velocity %i\n",
			front_left_wheel->GetEncVel());
	log->write(Log::INFO_LEVEL, "Get: Front Right Wheel Velocity %i\n",
			front_right_wheel->GetEncVel());
	log->write(Log::INFO_LEVEL, "Get: Back Left Wheel Velocity %i\n",
			rear_left_wheel->GetEncVel());
	log->write(Log::INFO_LEVEL, "Get: Back Right Wheel Velocity %i\n",
			rear_right_wheel->GetEncVel());

	log->write(Log::INFO_LEVEL, "Get: Lifter One Position %i\n",
			lifter_one->GetEncPosition());

	log->write(Log::INFO_LEVEL, "Get: Rake Port Position %i\n",
			rake_port->GetEncPosition());
	log->write(Log::INFO_LEVEL, "Get: Rake Starboard Position %i\n",
			rake_starboard->GetEncPosition());

	log->write(Log::INFO_LEVEL, "Get: Arm Flippy Floppies Position %i\n",
			flaps->GetEncPosition());

	log->write(Log::INFO_LEVEL, "Get: Upper Lifter Limit Switch %i\n",
			lifter_one->GetForwardLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Lower Lifter Limit Switch %i\n",
			lifter_one->GetReverseLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Upper Rake Port Limit Switch %i\n",
			rake_port->GetForwardLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Lower Rake Port Limit Switch %i\n",
			rake_port->GetReverseLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Upper Rake Starboard Limit Switch %i\n",
			rake_starboard->GetForwardLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Lower Rake Starboard Limit Switch %i\n",
			rake_starboard->GetReverseLimitOK());
	log->write(Log::INFO_LEVEL,
			"Get: Upper Arm Flippy Floppies isOpen Limit Switch %i\n",
			flaps->GetForwardLimitOK());
	log->write(Log::INFO_LEVEL,
			"Get: Lower Arm Flippy Floppies isClosed Limit Switch %i\n",
			flaps->GetReverseLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Arm potentiometer %i\n",
			flaps->GetAnalogIn());

}

void Motors::moveLifter() {
	PortsByMotor::Direction direction = ds->lift_motor->getDirection();
	if (direction == PortsByMotor::FORWARD) {
		lifter_one->Set(0.5);
		lifter_two->Set(0.5);
	}
	else if (direction == PortsByMotor::STILL) {
		lifter_one->Set(0.0);
		lifter_two->Set(0.0);
	}
	else if (direction == PortsByMotor::BACKWARD) {
		lifter_one->Set(-0.5);
		lifter_two->Set(-0.5);
	}
}

void Motors::setMotorDirection(PortsByMotor* ports, CANTalon* motor) {
	PortsByMotor::Direction direction = ports->getDirection();
	if (direction == PortsByMotor::FORWARD) {
		motor->Set(0.5);
	}
	else if (direction == PortsByMotor::STILL) {
		motor->Set(0.0);
	}
	else if (direction == PortsByMotor::BACKWARD) {
		motor->Set(-0.5);
	}
}

Motors* Motors::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new Motors();
	}
	return INSTANCE;
}
