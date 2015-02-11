/*
 * Motors.h
 *
 *  Created on: Feb 10, 2015
 *      Author: Lenovo
 */

#ifndef SRC_MOTORS_H_
#define SRC_MOTORS_H_

class Motors {
public:
	Motors();
	void process();

private:

	DS* ds;

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
};

#endif /* SRC_MOTORS_H_ */
