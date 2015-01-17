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

	/*if(grab_button->Get()){
		manipulator->grab();
=======
//	mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
//	mobility->setRotation(turn_direction_knob->Get());
	mobility->setDirection(main_joystick->GetRawAxis(4), main_joystick->GetRawAxis(5));
	mobility->setRotation(main_joystick->GetRawAxis(0));
/*	if(main_joystick->GetRawButton(1))
	{
		mobility->runTalon(RobotPorts::REAR_LEFT_MOTOR,0.5);
>>>>>>> Test-Branch
	}
	else if(!main_joystick->GetRawButton(1))
	{
		mobility->runTalon(RobotPorts::REAR_LEFT_MOTOR,0);
	}
	if(main_joystick->GetRawButton(2))
	{
		mobility->runTalon(RobotPorts::REAR_RIGHT_MOTOR,0.5);
	}
<<<<<<< HEAD
	else{
		manipulator->moveToHeight(1);
=======
	else if(!main_joystick->GetRawButton(2))
	{
		mobility->runTalon(RobotPorts::REAR_RIGHT_MOTOR,0);
	}
	if(main_joystick->GetRawButton(3))
	{
		mobility->runTalon(RobotPorts::FRONT_LEFT_MOTOR,0.5);
	}
	else if(!main_joystick->GetRawButton(3))
	{
		mobility->runTalon(RobotPorts::FRONT_LEFT_MOTOR,0);
	}
	if(main_joystick->GetRawButton(4))
	{
		mobility->runTalon(RobotPorts::FRONT_RIGHT_MOTOR,0.5);
	}
	else if(!main_joystick->GetRawButton(4))
	{
		mobility->runTalon(RobotPorts::FRONT_RIGHT_MOTOR,0);
>>>>>>> Test-Branch
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
