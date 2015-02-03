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
	if (!delay_timer->HasPeriodPassed(delay)) {
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
		++current_step;
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
		if (!timer->HasPeriodPassed(0.5)) {
			mobility->setDirection(0.0, 0.2);
		}
		else {
			timer->Reset();
			timer->Start();
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 8:
		// place the tote
		manipulator->closeFlaps(false);
		manipulator->pushTote();
		if (timer->HasPeriodPassed(0.5)) {
			++current_step;
			timer->Reset();
		}
		break;
	case 9:
		// back up
		if (timer->HasPeriodPassed(1.0)) {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		else {
			mobility->setDirection(0.0, -0.5);
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
	case 1:
		// moving to the container
		// assumes the robot is at a +90 degree angle to the landmark (facing container 1 from the left)
		manipulator->closeFlaps(false);
		if (mobility->getUltrasonicDistance() > 3) {
			mobility->setDirection(0.0, 0.5);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 2:
		// picking up the container
		manipulator->pullTote();
		manipulator->closeFlaps(true);
		manipulator->setTargetLevel(1);
		mobility->setRotationDegrees(90);
		++current_step;
		break;
	case 3:
		// move into the auto zone
		switch (starting_location) {
		case FAR_LEFT:
		case FAR_RIGHT:
			//this gets us aligned with the landmark
			if (mobility->getUltrasonicDistance() < 187) {
				mobility->setDirection(0.0, -0.5);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				++current_step;
			}
			break;
		case CENTER:
			// this gets us into the zone without running over the stack that might be built at the landmark
			if (mobility->getUltrasonicDistance() < 160) {
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
		// put down the container
		manipulator->setTargetLevel(0);
		if (manipulator->getLevel() == 0) {
			manipulator->closeFlaps(false);
			manipulator->pushTote();
			++current_step;
		}
		break;
	case 5:
		// backing up to be sure we aren't touching the container
		if (mobility->getUltrasonicDistance() < 14) {
			mobility->setDirection(0.0, -0.5);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 6:
		//yay done :D :D :D :D :D :D
		break;
	}
}

void Autonomous::moveContainerAndTote() {
	//assuming we're facing container, we pick up container, place it on tote, then move into autozone
	switch (current_step) {
	case 1:
		// moving to the container
		// assumes the robot is at a +90 degree angle to the landmark (facing container 1 from the left)
		manipulator->closeFlaps(false);
		if (mobility->getUltrasonicDistance() > 3) {
			mobility->setDirection(0.0, 0.5);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 2:
		// picking up the container
		manipulator->pullTote();
		manipulator->closeFlaps(true);
		manipulator->setTargetLevel(1);
		mobility->setRotationDegrees(90);
		++current_step;
		break;
	case 3:
		// moving to the tote
		if (mobility->getUltrasonicDistance() > 3) {
			mobility->setDirection(0.0, 0.5);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 4:
		// put container on top of tote
		if (manipulator->getLevel() != 1) {
			//wait for manipulator to lift to level one
			break;
		}
		else {
			manipulator->closeFlaps(false);
			manipulator->pushTote();	//pushing container, not tote, fyi
			++current_step;
		}
		break;
	case 5:
		//back up + lower manipulator lift
		mobility->setDirection(0.0, -0.2);
		manipulator->setTargetLevel(0);
		if (mobility->getUltrasonicDistance() > 10) { //change this number later, probably
			mobility->setDirection(0.0, 0.0);
			if (manipulator->getLevel() == 0) {
				++current_step;
			}
		}
		break;
	case 6:
		//scoot forward again :D
		if (mobility->getUltrasonicDistance() > 3) {
			mobility->setDirection(0.0, 0.5);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 7:
		//pick up tote/container stack
		manipulator->pullTote();
		manipulator->closeFlaps(true);
		manipulator->setTargetLevel(1);
		++current_step;
		break;
	case 8:
		//move into auto zone
		switch (starting_location) {
		case FAR_LEFT:
		case FAR_RIGHT:
			//this gets us aligned with the landmark
			if (mobility->getUltrasonicDistance() < 187) {
				mobility->setDirection(0.0, -0.5);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				++current_step;
			}
			break;
		case CENTER:
			// this gets us into the zone without running over the stack that might be built at the landmark
			if (mobility->getUltrasonicDistance() < 160) {
				mobility->setDirection(0.0, -0.5);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				++current_step;
			}
			break;
		}
		break;
	case 9:
		//put it down
		manipulator->setTargetLevel(0);
		if (manipulator->getLevel() == 0) {
			manipulator->closeFlaps(false);
			manipulator->pushTote();
			++current_step;
		}
		break;
	case 10:
		//back up so we're sure we aren't touching the tote/container
		if (mobility->getUltrasonicDistance() < 14) {
			mobility->setDirection(0.0, -0.5);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 11:
		//yay done :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D :D
		break;
	}
}

void Autonomous::centerContainers() {
	//facing wall, starting in between auto zone and landfill zone, move to pick up containers on step
	switch (current_step) {
	case 1:
		// move backwards towards step
		// 240 is a guess - we need to move back until the back of the robot is at the landfill
		if (mobility->getUltrasonicDistance() < 240) { //distance to alliance wall
			mobility->setDirection(0.0, -0.5);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 2:
		// extend rakes and hook the containers
		//manipulator->liftRakes(true); ???
		break;
	case 3:
		//move forward again to pull the containers off
		if (mobility->getUltrasonicDistance() > 220) {//distance to alliance wall
			mobility->setDirection(0.0, 0.5);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 4:
		//yay done :D
		break;
	}
}

void Autonomous::moveTwoTotes() {
	//move two tote stack into the auto zone on landmark
	switch (current_step) {
	case 1:
		// pick up 1st tote
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
		++current_step;
		break;
	case 3:
		// navigate the container - we move infield to move around it
		if (mobility->getUltrasonicDistance() < 30) {
			mobility->setDirection(0.5, 0.0);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 4:

		break;
	case 5:
		break;
	}
}

Autonomous* Autonomous::getInstance(int delay, int play, int location) {
	if (INSTANCE == NULL) {
		INSTANCE = new Autonomous(delay, play, location);
	}
	return INSTANCE;
}
