#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
#include "Log.h"
using namespace std;
DS* DS::INSTANCE = NULL;

DS::DS()
{
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();
	log = Log::getInstance();
	main_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	secondary_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	buttons = Joystick::GetStickForPort(DSPorts::BUTTONS_JOYSTICK);
	camForward = new USBCamera("Robot View 1", true);
	camBackwards = new USBCamera("Robot View 2", true);
	server = new CameraServer();
	on_step = false;
	backwards_camera = false;
	override = false;
}

void DS::process()
{
	if(secondary_joystick->GetRawButton(JoystickPorts::OVERRIDE_BUTTON)){
		log->write(Log::INFO_LEVEL,"Override button pressed");
		override=!override;
	}

	if(!override){
		//normal control by first driver
		mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
		mobility->setRotation(main_joystick->GetTwist());

		if(secondary_joystick->GetY()>0.25){
			manipulator->pushTote();
		}else if(secondary_joystick->GetY()<-0.25){
			manipulator->pullTote();
		}

		manipulator->spinTote(secondary_joystick->GetTwist());

	}
	else{
		//secondary driver has overriden so that they can control movement
		//I'm halving all input because this is for precision
		mobility->setDirection(secondary_joystick->GetX()/2.0,secondary_joystick->GetY()/2.0);
		mobility->setRotation(secondary_joystick->GetTwist()/2.0);
	}

	on_step = buttons->GetRawButton(ButtonPorts::STACK_ON_STEP_SWITCH);

	if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_1)){
		manipulator->setTargetHeight(1,on_step);
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_2)){
		manipulator->setTargetHeight(2,on_step);
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_3)){
		manipulator->setTargetHeight(3,on_step);
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_4)){
		manipulator->setTargetHeight(4,on_step);
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_5)){
		manipulator->setTargetHeight(5,on_step);
	}else if(buttons->GetRawButton(ButtonPorts::LIFTER_PRESET_6)){
		manipulator->setTargetHeight(6,on_step);
	}else{
		//do nothing
	}

	if(buttons->GetRawButton(ButtonPorts::MOVE_UP_BUTTON)){
		manipulator->changeHeight(0.5);
	}else if(buttons->GetRawButton(ButtonPorts::MOVE_DOWN_BUTTON)){
		manipulator->changeHeight(-0.5);
	}

	backwards_camera = buttons->GetRawButton(ButtonPorts::CAMERA_SELECT_TOGGLE);

	if(backwards_camera == false){
		startCameraForward();
	}
	else
		startCameraBackward();

}
void DS::startCameraForward(){
	camForward->UpdateSettings();
	camForward->OpenCamera();
	camForward->StartCapture();
	server->StartAutomaticCapture("Robot View 1");
}
void DS::startCameraBackward(){
	camBackwards->UpdateSettings();
	camBackwards->OpenCamera();
	camBackwards->StartCapture();
	server->StartAutomaticCapture();
}
DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
