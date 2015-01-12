#ifndef DS_H_
#define DS_H_

#include "Mobility.h"

class DS
{
public:
	static DS* getInstance();
	void process();

private:
	DS();
	static DS* INSTANCE;

	Joystick* test_gamepad;
	Mobility* mobility;
};

#endif // DS_H_
