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
	main_joystick = Joystick::GetStickForPort(DSPorts::MAIN_JOYSTICK);
	grab_button = new DigitalInput(DSPorts::GRAB_BUTTON);
	turn_direction_knob = new AnalogPotentiometer(DSPorts::TURN_DIRECTION_KNOB,2.0,-1.0);
	//IDK what exactly we want to do with knob so I'm doing this for now. values [-1,1]
	lifter_position_switch_up = new DigitalInput(DSPorts::LIFTER_POSITION_SWITCH_UP);
	lifter_position_switch_down = new DigitalInput(DSPorts::LIFTER_POSITION_SWITCH_DOWN);
	//using two digital inputs to model a three position switch


}

void DS::process()
{
	mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
	mobility->setRotation(turn_direction_knob->Get());

	if(grab_button->Get()){
		//TODO - do the grabby thing
	}

	if(lifter_position_switch_up->Get())
		manipulator->moveToHeight(2);
	else if(lifter_position_switch_down->Get())
		manipulator->moveToHeight(0);
	else
		manipulator->moveToHeight(1);

}

DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
