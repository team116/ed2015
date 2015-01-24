#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
#include "Log.h"
#include <CameraServer.h>
DS* DS::INSTANCE = NULL;

DS::DS()
{
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();
	log = Log::getInstance();
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

	if(secondary_joystick->GetRawButton(JoystickPorts::OVERRIDE_BUTTON)) {
		log->write(Log::INFO_LEVEL,"Override button pressed");
		override=!override;
	}

	if(digitalIO->GetRawButton(DigitalIOPorts::CAMERA_SELECT_TOGGLE)) {
		//switch to back camera
	}
	else {
		//switch to front camera
	}

	processMobility();
	processManipulator();
	processLEDS();

}

void DS::processMobility()
{
	if(override) {
		//secondary driver has overriden so that they can control movement
		//I'm halving all input because this is for precision
		//we might just remove this because the override button is a dumb idea
		mobility->setDirection(secondary_joystick->GetX()/2.0,secondary_joystick->GetY()/2.0);
		mobility->setRotation(secondary_joystick->GetTwist()/2.0);
	}
	else {
		//normal control by first driver
		drive_type = main_joystick->GetRawButton(JoystickPorts::FIELD_CENTRIC_TOGGLE);
		//check if the driver is trying to change to/from field-centric
		if(drive_type && !drive_type_handled) {
			log->write(Log::INFO_LEVEL, "Field-centric toggle pressed\n");
			drive_type_handled = true;
			mobility->toggleFieldCentric();
		}
		else if(drive_type_handled && !drive_type) {
			drive_type_handled = false;
		}

		mobility->setDirection(main_joystick->GetX(),main_joystick->GetY());
		mobility->setRotation(main_joystick->GetTwist());
	}
}

void DS::processManipulator()
{
	if(digitalIO->GetRawButton(DigitalIOPorts::STACK_ON_STEP_SWITCH)) {
		manipulator->setSurface(Manipulator::STEP);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::STACK_ON_PLATFORM_SWITCH)) {
		manipulator->setSurface(Manipulator::SCORING_PLATFORM);
	}
	else {
		manipulator->setSurface(Manipulator::FLOOR);
	}

	if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_1)) {
		manipulator->setTargetLevel(0);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_2)) {
		manipulator->setTargetLevel(1);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_3)) {
		manipulator->setTargetLevel(2);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_4)) {
		manipulator->setTargetLevel(3);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_5)) {
		manipulator->setTargetLevel(4);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_PRESET_6)) {
		manipulator->setTargetLevel(5);
	}
	else {
		//do nothing
	}

	if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_UP_BUTTON)) {
		manipulator->liftLifters(Manipulator::MOVING_UP);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::LIFTER_DOWN_BUTTON)) {
		manipulator->liftLifters(Manipulator::MOVING_DOWN);
	}
	else {
		manipulator->liftLifters(Manipulator::NOT_MOVING);
	}

	if(digitalIO->GetRawButton(DigitalIOPorts::RAKES_UP_BUTTON)) {
		manipulator->liftRakes(true);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::RAKES_DOWN_BUTTON)) {
		manipulator->liftRakes(false);
	}
	if(!override) {
		//normal control of manipulator by driver two
		if(secondary_joystick->GetY()>0.25) {
			manipulator->pushTote();
		}
		else if(secondary_joystick->GetY()<-0.25) {
			manipulator->pullTote();
		}

		manipulator->spinTote(secondary_joystick->GetTwist());
	}
}

void DS::processLEDS()
{
	digitalIO->SetOutputs(0);
	switch (manipulator->getLevel()) {
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

	if(digitalIO->GetRawButton(DigitalIOPorts::STACK_ON_PLATFORM_SWITCH)) {
		digitalIO->SetOutput(DigitalIOPorts::STACK_ON_PLATFORM_INDICATOR,true);
	}
	else if(digitalIO->GetRawButton(DigitalIOPorts::STACK_ON_STEP_SWITCH)) {
		digitalIO->SetOutput(DigitalIOPorts::STACK_ON_STEP_INDICATOR,true);
	}
	else {
		digitalIO->SetOutput(DigitalIOPorts::STACK_ON_FLOOR_INDICATOR,true);
	}

	if(digitalIO->GetRawButton(DigitalIOPorts::CAMERA_SELECT_TOGGLE)){
		digitalIO->SetOutput(DigitalIOPorts::BACK_CAMERA_INDICATOR,true);
	}
	else {
		digitalIO->SetOutput(DigitalIOPorts::FRONT_CAMERA_INDICATOR,true);
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
