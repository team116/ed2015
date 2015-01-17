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


};

#endif // DS_H_
