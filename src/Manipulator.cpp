#include <WPILib.h>
#include <CANTalon.h>
#include "Manipulator.h"
#include "Ports.h"

Manipulator* Manipulator::INSTANCE = NULL;

Manipulator::Manipulator()
{
	mobility = Mobility::getInstance();

	left_wheel = new CANTalon(RobotPorts::LEFT_WHEEL);
	right_wheel = new CANTalon(RobotPorts::RIGHT_WHEEL);
	lifter_one = new CANTalon(RobotPorts::LIFTER_ONE);
	lifter_two = new CANTalon(RobotPorts::LIFTER_TWO);
	rake_port = new CANTalon(RobotPorts::RAKE_PORT);
	rake_starboard = new CANTalon(RobotPorts::RAKE_STARBOARD);
	current_height = 0; //starting height (floor level)
	belt_moving = false;

}

Manipulator::~Manipulator() {
	// TODO Auto-generated destructor stub
}


Manipulator* Manipulator::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new Manipulator();
	}
	return INSTANCE;
}


void Manipulator::process() {

}


void Manipulator::grab()
{
	left_wheel ->Set(0.5);
	right_wheel->Set(0.5);

	//do the grabbbbbbbby thing

}
void Manipulator::closeArms(bool close) {

}
void Manipulator::moveToHeight(int level) {

}
float Manipulator::getHeight(){
	return current_height;
}
void Manipulator::startConveyorBelt() {
	belt_moving = true;
}
void Manipulator::stopConveyorBelt() {
	belt_moving = false;
}
void Manipulator::honor_limits(bool to_use_or_not_to_use) {
	using_limits = to_use_or_not_to_use;
}
void Manipulator::read_limits() {
	//some_limit = arduino->get_some_limit();
}
void Manipulator::liftLifters()
{
	lifter_one ->Set(0.5);
	lifter_two->Set(0.5);
}
void Manipulator::liftRakes()
{
	rake_port ->Set(0.5);
	rake_starboard ->Set(0.5);
	//controls moving rakes up/down
}
void Manipulator::spinWheels()
{

}
