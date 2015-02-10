#include <WPILib.h>
#include <Timer.h>
#include "Ports.h"
#include "Autonomous.h"
#include "Mobility.h"
#include "Log.h"
#include "Manipulator.h"

Autonomous* Autonomous::INSTANCE = NULL;
// approximate speed while running at 0.75
// if we can get an accurate value here then timers will provide a safety net if the ultrasonic fails
const float Autonomous::INCHES_PER_SECOND = 30.0;

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
	timer->Start();
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
		if (mobility->getUltrasonicDistance() < 163 && mobility->getYEncoderDistance() < 140 && !timer->HasPeriodPassed(140.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Stop();
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
		// assumes the robot is at a 90 degree angle to the landmark (facing tote 1 from the right)
		manipulator->closeFlaps(false);
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(12.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 2:
		// picking up the tote
		if(!timer->HasPeriodPassed(1.0)){
			manipulator->pullTote();
		}
		else {
			manipulator->closeFlaps(true);
			manipulator->setTargetLevel(2);
			timer->Reset();
			// turn the robot so that it is facing the alliance wall
			mobility->setRotationDegrees(90);
			++current_step;
		}
		break;
	case 3:
		// moving to autozone
		switch (starting_location) {
		case FAR_LEFT:
		case FAR_RIGHT:
			if (mobility->getUltrasonicDistance() < 187 && mobility->getYEncoderDistance() < 140 && !timer->HasPeriodPassed(140.0/INCHES_PER_SECOND)) {
				mobility->setDirection(0.0, -0.75);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				timer->Reset();
				mobility->resetYEncoderDistance();
				++current_step;
			}
			break;
		case CENTER:
			if (mobility->getUltrasonicDistance() < 148 && mobility->getYEncoderDistance() < 100 && !timer->HasPeriodPassed(100.0/INCHES_PER_SECOND)) {
				mobility->setDirection(0.0, -0.75);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				timer->Reset();
				mobility->resetYEncoderDistance();
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
			if (mobility->getUltrasonicDistance() < 160 && mobility->getYEncoderDistance() < 53 && !timer->HasPeriodPassed(53.0/INCHES_PER_SECOND)) {
				mobility->setDirection(0.0, -0.75);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				timer->Reset();
				timer->Stop();
				mobility->resetYEncoderDistance();
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
		case FAR_RIGHT:
			// turn towards landmark
			mobility->setRotationDegrees(180);
			++current_step;
			break;
		case CENTER:
			// still nothing to do yet
			++current_step;
			break;
		}
		// reset/start a timer so that we can move forward for a brief time
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
		// wait a brief moment to be sure that we've actually placed the tote
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
			mobility->setDirection(0.0, -0.75);
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
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(12.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 2:
		// picking up the container
		manipulator->pullTote();
		if(timer->HasPeriodPassed(1.0)){
			manipulator->closeFlaps(true);
			manipulator->setTargetLevel(1);
			// turn to face the alliance wall
			mobility->setRotationDegrees(90);
			timer->Reset();
			++current_step;
		}
		break;
	case 3:
		// move into the auto zone
		switch (starting_location) {
		case FAR_LEFT:
		case FAR_RIGHT:
			//this gets us aligned with the landmark
			if (mobility->getUltrasonicDistance() < 187 && mobility->getYEncoderDistance() < 140 && !timer->HasPeriodPassed(140.0/INCHES_PER_SECOND)) {
				mobility->setDirection(0.0, -0.75);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				timer->Reset();
				mobility->resetYEncoderDistance();
				++current_step;
			}
			break;
		case CENTER:
			// this gets us into the zone without running over the stack that might be built at the landmark
			if (mobility->getUltrasonicDistance() < 160 && mobility->getYEncoderDistance() < 100 && !timer->HasPeriodPassed(100.0/INCHES_PER_SECOND)) {
				mobility->setDirection(0.0, -0.75);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				mobility->resetYEncoderDistance();
				timer->Reset();
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
		if (mobility->getUltrasonicDistance() < 14 && mobility->getYEncoderDistance() < 16 && !timer->HasPeriodPassed(16.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			mobility->resetYEncoderDistance();
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
		//sam chanesman
		manipulator->closeFlaps(false);
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(12/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 2:
		// picking up the container
		manipulator->pullTote();
		if(timer->HasPeriodPassed(1.0)){
			manipulator->closeFlaps(true);
			manipulator->setTargetLevel(1);
			mobility->setRotationDegrees(90);
			timer->Reset();
			++current_step;
		}
		break;
	case 3:
		// moving to the tote
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 18 && !timer->HasPeriodPassed(18.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Stop();
			timer->Reset();
			mobility->resetYEncoderDistance();
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
		// back up + lower manipulator lift
		manipulator->setTargetLevel(0);
		// one foot is an arbitrary distance to back up
		if (mobility->getUltrasonicDistance() < 12 && mobility->getYEncoderDistance() < 14 && !timer->HasPeriodPassed(14.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			if (manipulator->getLevel() == 0) {
				// move on if we've backed up enough and the manipulator has lowered
				++current_step;
			}
		}
		break;
	case 6:
		//scoot forward again :D
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 14 && !timer->HasPeriodPassed(14.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 7:
		//pick up tote/container stack
		manipulator->pullTote();
		if(timer->HasPeriodPassed(1.0)){
			manipulator->closeFlaps(true);
			manipulator->setTargetLevel(1);
			timer->Reset();
			++current_step;
		}
		break;
	case 8:
		//move into auto zone
		switch (starting_location) {
		case FAR_LEFT:
		case FAR_RIGHT:
			//this gets us aligned with the landmark
			if (mobility->getUltrasonicDistance() < 187 && mobility->getYEncoderDistance() < 140 && !timer->HasPeriodPassed(140.0/INCHES_PER_SECOND)) {
				mobility->setDirection(0.0, -0.75);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				timer->Reset();
				mobility->resetYEncoderDistance();
				++current_step;
			}
			break;
		case CENTER:
			// this gets us into the zone without running over the stack that might be built at the landmark
			if (mobility->getUltrasonicDistance() < 160 && mobility->getYEncoderDistance() < 100 &&!timer->HasPeriodPassed(100.0/INCHES_PER_SECOND)) {
				mobility->setDirection(0.0, -0.75);
			}
			else {
				mobility->setDirection(0.0, 0.0);
				timer->Reset();
				mobility->resetYEncoderDistance();
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
			timer->Reset();
			++current_step;
		}
		break;
	case 10:
		//back up so we're sure we aren't touching the tote/container
		if (mobility->getUltrasonicDistance() < 14 && mobility->getYEncoderDistance() < 14 && !timer->HasPeriodPassed(14.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			mobility->resetYEncoderDistance();
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
		if (mobility->getUltrasonicDistance() < 240 && mobility->getYEncoderDistance() < 24 && !timer->HasPeriodPassed(24.0/INCHES_PER_SECOND)) { //distance to alliance wall
			mobility->setDirection(0.0, -0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 2:
		// extend rakes and hook the containers
		manipulator->liftRakes(true);
		// wait a moment to allow the rakes to do their thing. this might need to be longer
		if(timer->HasPeriodPassed(2.5)){
			++current_step;
		}
		break;
	case 3:
		//move forward again to pull the containers off
		if (mobility->getUltrasonicDistance() > 220 && mobility->getYEncoderDistance() < 30 && !timer->HasPeriodPassed(30.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
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
	// move two tote stack into the auto zone on landmark
	switch (current_step) {
	case 1:
		// scoot in towards tote
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(12.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 2:
		// picking up the tote
		// wait to ensure that the tote has actually been pulled in
		if (timer->HasPeriodPassed(1.5)) {
			timer->Reset();
			manipulator->closeFlaps(true);
			manipulator->setTargetLevel(1);
			++current_step;
		}
		else {
			manipulator->pullTote();
		}
		break;
	case 3:
		// navigate the container - we move infield to move around it
		if (mobility->getUltrasonicDistance() < 30 && mobility->getXEncoderDistance() < 36 && !timer->HasPeriodPassed(36.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.5, 0.0);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetXEncoderDistance();
			++current_step;
		}
		break;
	case 4:
		// move until we're in position to move in and grab the center tote
		// ok so this is a horrible estimate
		// but hopefully this gets us just behind the center tote
		if (mobility->getUltrasonicDistance() > 188.0) {
			mobility->setDirection(0.0, 0.75);
		}
		else if (mobility->getUltrasonicDistance() < 184.0) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			++current_step;
		}
		break;
	case 5:
		// scoot in sideways so that we can grab the tote
		if (mobility->getUltrasonicDistance() > 15 && mobility->getXEncoderDistance() < 36 && !timer->HasPeriodPassed(36.0/INCHES_PER_SECOND)) {
			mobility->setDirection(-0.5, 0.0);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetXEncoderDistance();
			++current_step;
		}
		break;
	case 6:
		// scoot forward to second tote
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 10 && !timer->HasPeriodPassed(10.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Start();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 7:
		// place the first tote on top of the second
		manipulator->closeFlaps(false);
		manipulator->pushTote();
		// wait briefly to make sure we actually pushed the tote out
		if (timer->HasPeriodPassed(1.0)) {
			timer->Reset();
			++current_step;
		}
		break;
	case 8:
		// back up + lower manipulator lift
		manipulator->setTargetLevel(0);
		// one foot is an arbitrary distance to back up
		if (mobility->getUltrasonicDistance() < 12 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(12.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			if (manipulator->getLevel() == 0) {
				// move on if we've backed up enough and the manipulator has lowered
				timer->Reset();
				mobility->resetYEncoderDistance();
				++current_step;
			}
		}
		break;
	case 9:
		//scoot forward again
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(12.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 10:
		//pick up the two stacked totes and start lifting them
		// wait to ensure that the totes have actually been pulled in
		if (timer->HasPeriodPassed(1.5)) {
			timer->Reset();
			manipulator->closeFlaps(true);
			manipulator->setTargetLevel(1);
			// turn to face the alliance wall
			mobility->setRotationDegrees(90);
			++current_step;
		}
		else {
			manipulator->pullTote();
		}
		break;
	case 11:
		// moving to auto zone
		if (mobility->getUltrasonicDistance() < 148 && mobility->getYEncoderDistance() < 100 && !timer->HasPeriodPassed(100.0/INCHES_PER_SECOND)) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		break;
	case 12:
		//turn to face landmark
		mobility->setRotationDegrees(180);
		//reset/start a timer so that we can move forward briefly to get very close to the landmark
		timer->Reset();
		++current_step;
		break;
	case 13:
		// scoot forward towards the landmark
		// if we know we're stacking our totes after another bot goes then we can improve this like so
		/*
		 if (mobility->getUltrasonicDistance() > 3) {
		 mobility->setDirection(0.0, 0.75);
		 }
		 else {
		 mobility->setDirection(0.0, 0.0);
		 timer->Reset();
		 timer->Start();
		 ++current_step;
		 }
		 break;
		 */

		// but for now I'll do this lame timer thing
		if (!timer->HasPeriodPassed(1.0)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			timer->Reset();
			timer->Start();
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 14:
		// place the tote
		manipulator->closeFlaps(false);
		manipulator->pushTote();
		if (timer->HasPeriodPassed(0.5)) {
			++current_step;
			timer->Reset();
		}
		break;
	case 15:
		// back up
		if (timer->HasPeriodPassed(1.0)) {
			mobility->setDirection(0.0, 0.0);
			timer->Stop();
			++current_step;
		}
		else {
			mobility->setDirection(0.0, -0.75);
		}
		break;
	case 16:
		// yay we're done
		break;
	}

}

Autonomous* Autonomous::getInstance(int delay, int play, int location) {
	if (INSTANCE == NULL) {
		INSTANCE = new Autonomous(delay, play, location);
	}
	return INSTANCE;
}
