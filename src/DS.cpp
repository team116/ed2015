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
}

void DS::process()
{
	mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
	mobility->setRotation(main_joystick->GetTwist());

	/*if(grab_button->Get()){
		manipulator->grab();
	}

	if(lifter_position_switch_up->Get()){
		manipulator->moveToHeight(2);
	}
	else if(lifter_position_switch_down->Get()){
		manipulator->moveToHeight(0);
	}
	else{
		manipulator->moveToHeight(1);
	}*/
}

DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
