/*
 * PortsByMotor.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: strahans
 */

#include "PortsByMotor.h"
#include <Joystick.h>

PortsByMotor::PortsByMotor(int joystick_port_1, int joystick_port_2, Joystick* joy) {
	PORT_1 = joystick_port_1;
	PORT_2 = joystick_port_2;
	joystick = joy;
}

PortsByMotor::Direction PortsByMotor::getDirection() {
	if (joystick->GetRawButton(PORT_1)) {
		if (!joystick->GetRawButton(PORT_2)) {
			return FORWARD;
		}
	}
	else if (joystick->GetRawButton(PORT_2)) {
		return BACKWARD;
	}
	return STILL;
}



