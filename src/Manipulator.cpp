#include "Manipulator.h"
#include "WPILib.h"
#include "Ports.h"

Manipulator* Manipulator::INSTANCE = NULL;

Manipulator::Manipulator() {
	height = 0; //starting height (floor level)
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


void Manipulator::grab() {
	//do the grabbbbbbbby thing

}
void Manipulator::closeArms() {

}
void Manipulator::moveToHeight(int level) {

}
int Manipulator::getHeight(){
	return height;
}
void Manipulator::openArms() {

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
