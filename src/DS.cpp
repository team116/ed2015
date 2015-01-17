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
	buttons = Joystick::GetStickForPort(DSPorts::BUTTONS_JOYSTICK);
	lifter_preset = -6;
	on_step = false;
	backwards_camera = false;
	override = false;
}

void DS::process()
{
	if(secondary_joystick->GetRawButton(JoystickPorts::OVERRIDE_BUTTON)){
		override=!override;
	}

	if(!override){
		//normal control by first driver
		mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
		mobility->setRotation(main_joystick->GetTwist());

		if(secondary_joystick->GetY()>0.25){
			//release a tote
		}else if(secondary_joystick->GetY()<-0.25){
			//pull in tote
		}

		//manipulator->spinTote(secondary_joystick->GetTwist());

	}
	else{
		//secondary driver has overriden so that they can control
		//I'm halving all input because this is for precise control
		mobility->setDirection(secondary_joystick->GetX()/2.0,secondary_joystick->GetY()/2.0);
		mobility->setRotation(secondary_joystick->GetTwist()/2.0);
	}

	on_step = buttons->GetRawButton(ButtonPorts::STACK_ON_STEP_SWITCH);

	if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_1)){
		lifter_preset = 1;
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_2)){
		lifter_preset = 2;
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_3)){
		lifter_preset = 3;
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_4)){
		lifter_preset = 4;
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_5)){
		lifter_preset = 5;
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_6)){
		lifter_preset = 6;
	}else{
		lifter_preset = -6;
	}

	//manipulator->moveToPreset(lifter_preset,on_step);

	if(buttons->GetRawButton(ButtonPorts::MOVE_UP_BUTTON)){
		//manipulator->moveUp();
	}else if(buttons->GetRawButton(ButtonPorts::MOVE_DOWN_BUTTON)){
		//manipulator->moveDown();
	}

	backwards_camera = buttons->GetRawButton(ButtonPorts::CAMERA_SELECT_TOGGLE);
}

DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
