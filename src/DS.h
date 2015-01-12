#ifndef DS_H_
#define DS_H_

#include "Mobility.h"
#include "Manipulator.h"

class DS
{
public:
	static DS* getInstance();
	void process();

private:
	DS();
	static DS* INSTANCE;
	Mobility* mobility;
	Manipulator* manipulator;

	Joystick* main_joystick;
	DigitalInput* grab_button;
	AnalogPotentiometer* turn_direction_knob;
	AnalogPotentiometer* lifter_position_switch;


};

#endif // DS_H_
