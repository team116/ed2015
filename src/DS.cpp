#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
using namespace std;
DS* DS::INSTANCE = NULL;

DS::DS()
{
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();
	main_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	secondary_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	//IDK what exactly we want to do with knob so I'm doing this for now. values [-1,1]
	//using two digital inputs to model a three position switch
}

void DS::process()
{
	mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
	mobility->setRotation(main_joystick->GetTwist());
}

DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
