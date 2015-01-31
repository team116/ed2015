#include <WPILib.h>
#include <Timer.h>
#include "Ports.h"
#include "Autonomous.h"
#include "Mobility.h"
#include "Log.h"
#include "Manipulator.h"

Autonomous* Autonomous::INSTANCE = NULL;

Autonomous::Autonomous(int delay, int play, int location) {
	current_step = 1;
	starting_location = location;
	this->play = play;
	this->delay = delay;

	delay_timer = new Timer();
	timer = new Timer();
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();
	log = Log::getInstance();

	delay_timer->Start();
}

void Autonomous::process() {
	//wait for iiiiiiiiittt...
	if(!delay_timer->HasPeriodPassed(delay)){
		return;
	}
	//BOOM
	switch (play) {
	case Plays::DO_NOTHING:
		doNothing();
		break;
	case Plays::INTO_ZONE:
		moveToZone();
		break;
	case Plays::STACK_TOTE:
		stackTote();
		break;
	case Plays::MOVE_CONTAINER:
		moveContainer();
		break;
	case Plays::CONTAINER_AND_TOTE:
		moveContainerAndTote();
		break;
	}
}

void Autonomous::doNothing() {
	// nada
}

void Autonomous::moveToZone() {
	switch (current_step) {
	case 1:
		//163 inches is the distance from the wall to the autozone
		if (mobility->getUltrasonicDistance() < 163) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	default:
		break;
	}
}

void Autonomous::stackTote() {
	// picks up one tote, moves it to the landmark, and stacks it
	switch (current_step) {
	case 1:
		// moving to the tote
		// assumes the robot is at a -90 degree angle to the landmark (facing tote 1 from the right)
		manipulator->closeFlaps(false);
		// 35 or 3 inches away from tote?
		if (mobility->getUltrasonicDistance() > 3) {
			mobility->setDirection(0.0, 0.5);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 2:
		// picking up the tote
		manipulator->closeFlaps(true);
		manipulator->setTargetLevel(1);
		// turn the robot so that it is facing the alliance wall
		mobility->setRotationDegrees(-90);
		break;
	case 3:
		// moving to autozone
		switch (starting_location) {
		case FAR_LEFT:
		case FAR_RIGHT:
			if (mobility->getUltrasonicDistance() < 187) {
				mobility->setDirection(0.0, -0.5);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				++current_step;
			}
			break;
		case CENTER:
			if (mobility->getUltrasonicDistance() < 148) {
				mobility->setDirection(0.0, -0.5);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				++current_step;
			}
			break;
		}
		break;
	case 4:
		// turning in appropriate directions
		switch (starting_location) {
		case FAR_LEFT:
			//turn to face wall so we can judge distance as we approach landmark
			mobility->setRotationDegrees(-90);
			++current_step;
			break;
		case FAR_RIGHT:
			//turn to face wall so we can judge distance as we approach landmark
			mobility->setRotationDegrees(90);
			++current_step;
			break;
		case CENTER:
			//turn to face landmark
			mobility->setRotationDegrees(180);
			++current_step;
			break;
		}
		break;
	case 5:
		// moving to landmark
		switch (starting_location) {
		case FAR_LEFT:
		case FAR_RIGHT:
			//move forward
			if (mobility->getUltrasonicDistance() < 160) {
				mobility->setDirection(0.0, -0.5);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				++current_step;
			}
			break;
		case CENTER:
			//nothing to do in this step - we're already right by the landmark
			++current_step;
			break;
		}
		break;
	case 6:
		// turning for the final time
		switch (starting_location) {
		case FAR_LEFT:
			//turn towards landmark
			mobility->setRotationDegrees(180);
			++current_step;
			break;
		case FAR_RIGHT:
			//turn towards landmark
			mobility->setRotationDegrees(180);
			++current_step;
			break;
		case CENTER:
			//still nothing to do yet
			++current_step;
			break;
		}
		//reset/start a timer so that we can move forward for a brief time
		timer->Reset();
		timer->Start();
		break;
	case 7:
		// scoot forward towards the landmark
		// we can tweak the time and the speed to move us in the appopriate amount
		if(!timer->HasPeriodPassed(0.5)){
			mobility->setDirection(0.0,0.2);
		}
		else {
			timer->Reset();
			timer->Start();
			mobility->setDirection(0.0,0.0);
			++current_step;
		}
	case 8:
		// place the tote
		manipulator->pushTote();
		if(timer->HasPeriodPassed(0.5)){
			++current_step;
			timer->Reset();
		}
		break;
	case 9:
		// back up
		if(timer->HasPeriodPassed(1.0)){
			mobility->setDirection(0.0,0.0);
			++current_step;
		}
		else {
			mobility->setDirection(0.0,-0.5);
		}
		break;
	case 10:
		// yay we're done
		break;
	default:
		break;
	}
}

void Autonomous::moveContainer() {
	switch (current_step) {

	}
}

void Autonomous::moveContainerAndTote() {
	switch (current_step) {

	}
}

void Autonomous::centerContainers() {
	switch (current_step) {

	}
}

Autonomous* Autonomous::getInstance(int delay, int play, int location) {
	if (INSTANCE == NULL) {
		INSTANCE = new Autonomous(delay, play, location);
	}
	return INSTANCE;
}
