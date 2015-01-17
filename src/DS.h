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
	Log* log;

	Joystick* main_joystick;
	Joystick* secondary_joystick;
	Joystick* buttons;

	bool override;
	bool backwards_camera;
	bool on_step;


};

#endif // DS_H_
