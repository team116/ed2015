#ifndef DS_H_
#define DS_H_

#include "Mobility.h"
#include "Manipulator.h"
#include <CameraServer.h>
#include <USBCamera.h>


class DS
{
public:
	static DS* getInstance();
	void process();
	void startCameraForward();
	void startCameraBackward();

private:
	DS();
	static DS* INSTANCE;

	Mobility* mobility;
	Manipulator* manipulator;
	Log* log;
	CameraServer* server;
	USBCamera* camForward;
	USBCamera* camBackwards;

	Joystick* main_joystick;
	Joystick* secondary_joystick;
	Joystick* buttons;

	bool override;
	bool backwards_camera;
	bool on_step;

	const static float LIFTER_BUTTON_CHANGE;

};

#endif // DS_H_
