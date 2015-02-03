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
	void processMobility();
	void processManipulator();
	void processLEDS();

private:
	DS();
	static DS* INSTANCE;
	Mobility* mobility;
	Manipulator* manipulator;
	Log* log;
	CameraServer* server;

	Joystick* main_joystick;
	Joystick* secondary_joystick;
	Joystick* IO_board_one;
	Joystick* IO_board_two;

	bool override;
	bool on_step;
	bool drive_type;
	bool drive_type_handled;
	bool turn_degrees;
	bool turn_degrees_handled;

	void doLevelLEDS(int level);

	const static float LIFTER_BUTTON_CHANGE;

};

#endif // DS_H_
