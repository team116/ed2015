/*
 * PortsByMotor.h
 *
 *  Created on: Feb 14, 2015
 *      Author: strahans
 */

#ifndef SRC_PORTSBYMOTOR_H_
#define SRC_PORTSBYMOTOR_H_

#include <Joystick.h>

class PortsByMotor {
public:
	enum Direction {
		FORWARD,
		STILL,
		BACKWARD
	};

	PortsByMotor(int joystick_port_1, int joystick_port_2, Joystick* joystick);

	Direction getDirection();
	int PORT_1;
	int PORT_2;
	Joystick* joystick;
};

#endif /* SRC_PORTSBYMOTOR_H_ */
