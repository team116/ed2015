#ifndef DS_H_
#define DS_H_

#include "Mobility.h"
#include "Manipulator.h"
#include <CameraServer.h>

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
	CameraServer* server;

	Joystick* main_joystick;
	Joystick* secondary_joystick;
	Joystick* buttons;

	bool override;
	bool on_step;
	bool drive_type;
	bool drive_type_handled;

	const static float LIFTER_BUTTON_CHANGE;

};

#endif // DS_H_
