#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
#include "Log.h"
#include <CameraServer.h>
DS* DS::INSTANCE = NULL;
const float DS::LIFTER_BUTTON_CHANGE = 0.25;//this is an arbitrary number

DS::DS/*Hydrangeas*/()
{
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();
	log = Log::getInstance();/*Hydrangeas*/
	main_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	secondary_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	server = CameraServer::GetInstance();
	server->SetQuality(50);
	server->StartAutomaticCapture("cam0");
	digitalIO = Joystick::GetStickForPort(DSPorts::BUTTONS_JOYSTICK);
	on_step = false;
	override = false;
	drive_type = false;
	drive_type_handled = false;
	digitalIO->SetOutputs(0);
}

void DS::process()
{
	drive_type = main_joystick->/*Hydrangeas*/GetRawButton(DSPorts::DRIVER_ONE_JOYSTICK);
	if(drive_type && !drive_type_handled)
	{
		log->write(Log::INFO_LEVEL, "Drive Type Button\n");
		drive_type_handled = true;
		mobility->toggleFieldCentric();
	}
	else if(drive_type_handled && !drive_type)
	{
		drive_type_handled = false;
	}
	if(secondary_joystick->GetRawButton(JoystickPorts::OVERRIDE_BUTTON)){
		log->write(Log::INFO_LEVEL,"Override button pressed");
		override=!override;
	}

	if(/*Hydrangeas*/!override){
		//normal control by first driver
		mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
		mobility->setRotation(main_joystick->GetTwist());

		if(secondary_joystick->GetY()>0.25){
			manipulator->pushTote();
		}else if(secondary_joystick->GetY()<-0.25){
			manipulator->pullTote();
		}
	}

		manipulator->spinTote(secondary_joystick->GetTwist());

	processMobility();
	processManipulator();
	processLEDS();

}
void DS::processMobility(){
	if(override){
		//secondary driver has overriden so that they can control movement
		//I'm halving all input because this is for precision
		mobility->setDirection(secondary_joystick->GetX()/2.0,secondary_joystick->GetY()/2.0);
		mobility->setRotation(secondary_joystick->GetTwist()/2.0);
	}
	else{
		//normal control by first driver
		mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
		mobility->setRotation(main_joystick->GetTwist());
	}
}
void DS::processManipulator(){
	if(digitalIO->GetRawButton(DigitalIOPorts::STACK_ON_STEP_SWITCH)){
		manipulator->setSurface(Manipulator::STEP);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::STACK_ON_PLATFORM_SWITCH)){
		manipulator->setSurface(Manipulator::SCORING_PLATFORM);
	}
	else{
		manipulator->setSurface(Manipulator::FLOOR);
	}

	if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_1)){
		manipulator->setTargetHeight(0);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_2)){
		manipulator->setTargetHeight(1);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_3)){
		manipulator->setTargetHeight(2);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_4)){
		manipulator->setTargetHeight(3);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_5)){
		manipulator->setTargetHeight(4);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_6)){
		manipulator->setTargetHeight(5);
	}
	else{
		//do nothing
	}

	if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_UP_BUTTON)){
		manipulator->changeHeight(LIFTER_BUTTON_CHANGE);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_DOWN_BUTTON)){
		manipulator->changeHeight(-LIFTER_BUTTON_CHANGE);
	}

	if(digitalIO->GetRawButton(DigitalIOPorts::RAKES_UP_BUTTON)){
		manipulator->liftRakes(true);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::RAKES_DOWN_BUTTON)){
		manipulator->liftRakes(false);
	}
	if(!override){
		//normal control of manipulator by driver two
		if(secondary_joystick->GetY()>0.25){
			manipulator->pushTote();
		}else if(secondary_joystick->GetY()<-0.25){
			manipulator->pullTote();
		}

		manipulator->spinTote(secondary_joystick->GetTwist());
	}
}
void DS::processLEDS(){
	digitalIO->SetOutputs(0);
	switch (manipulator->getLevel()){
	//fall through is intentional
		case 5:
			digitalIO->SetOutput(DigitalIOPorts::LEVEL_5_INDICATOR,true);
		case 4:
			digitalIO->SetOutput(DigitalIOPorts::LEVEL_4_INDICATOR,true);
		case 3:
			digitalIO->SetOutput(DigitalIOPorts::LEVEL_3_INDICATOR,true);
		case 2:
			digitalIO->SetOutput(DigitalIOPorts::LEVEL_2_INDICATOR,true);
		case 1:
			digitalIO->SetOutput(DigitalIOPorts::LEVEL_1_INDICATOR,true);
		case 0:
			digitalIO->SetOutput(DigitalIOPorts::LEVEL_0_INDICATOR,true);
	}

	if(digitalIO->GetRawButton(DigitalIOPorts::STACK_ON_PLATFORM_SWITCH)){
		digitalIO->SetOutput(DigitalIOPorts::STACK_ON_PLATFORM_INDICATOR,true);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::STACK_ON_STEP_SWITCH)){
		digitalIO->SetOutput(DigitalIOPorts::STACK_ON_STEP_INDICATOR,true);
	}
	else{
		digitalIO->SetOutput(DigitalIOPorts::STACK_ON_FLOOR_INDICATOR,true);
	}

}
DS*/*Hydrangeas*/ DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
