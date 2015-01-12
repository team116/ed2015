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
	lifter_position_switch = new AnalogPotentiometer(DSPorts::LIFTER_POSITION_SWITCH,3.0,0);
	//again, IDK. values [0,3]


}

void DS::process()
{
	mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
	mobility->setRotation(turn_direction_knob->Get());

	if(grab_button->Get()){
		//TODO - do the grabby thing
	}

	int target_height = lifter_position_switch->Get();
	if(manipulator->getHeight()!=target_height){
		manipulator->moveToHeight(target_height);
	}

}

DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
