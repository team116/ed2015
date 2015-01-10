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
