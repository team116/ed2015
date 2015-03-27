#include <WPILib.h>
#include <Timer.h>
#include "Ports.h"
#include "Autonomous.h"
#include "Mobility.h"
#include "Log.h"
#include "Manipulator.h"

// timeout for going from start position to the first tote
const float Autonomous::FIRST_TOTE_TIMEOUT = 0.4;
// timeout for pulling in a tote/stack
const float Autonomous::PULL_TIMEOUT = 0.8;
// timeout for pushing out a tote/stack
const float Autonomous::PUSH_TIMEOUT = 0.6;
// timeout for moving from starting position into auto zone
const float Autonomous::MOVE_TO_ZONE_TIMEOUT = 1.3;
// timeout for backing up from a tote
const float Autonomous::BACKUP_TIMEOUT = 0.4;

Autonomous::Autonomous(int delay, int play, int location) {
	log = Log::getInstance();

	current_step = 1;
	starting_location = location;
	this->play = play;
	this->delay = delay;
	log->write(Log::ERROR_LEVEL, "Play: %d\nDelay: %d\nLocation: %d\n", play, delay, location);

	delay_timer = new Timer();
	timer = new Timer();
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();

	delay_over = false;
	delay_timer->Start();
	timer->Start();
}

void Autonomous::process() {
	//wait for iiiiiiiiittt...
/*	if (!delay_over) {
		if (!delay_timer->HasPeriodPassed(delay)) {
			return;
		}
		delay_over = true;
		log->write(Log::INFO_LEVEL, "%s\tAuto: delay over\n", Utils::getCurrentTime());
	}*/
	//BOOM
	switch (play) {
	case Plays::DO_NOTHING:
		doNothing();
		break;
	case Plays::INTO_ZONE:
		moveToZone();
		break;
	case Plays::STACK_THREE_TOTES://The big play
		moveThreeTotes();
		break;
	case Plays::MOVE_CONTAINER:
		moveContainer();
		break;
	case Plays::CONTAINER_AND_TOTE://Test odometry
		moveContainerAndTote();
		break;
	case Plays::CENTER_CONTAINERS:
		centerContainers();
		break;
	case Plays::STACK_TOTE:
		stackTote();
		break;
	default:
		log->write(Log::ERROR_LEVEL, "%s\tUnrecognized play value: %d\n", Utils::getCurrentTime(), play);
	}
}

void Autonomous::doNothing() {
	// nada
	switch (current_step) {
	case 1:
		log->write(Log::INFO_LEVEL, "%s\tAuto: Do Nothing started\n", Utils::getCurrentTime());
		++current_step;
		break;
	case 2:
		log->write(Log::INFO_LEVEL, "%s\tAuto: Do Nothing ended\n", Utils::getCurrentTime());
		++current_step;
		break;
	default:
		break;
	}
}

void Autonomous::moveToZone() {
	switch (current_step) {
	case 1:
		log->write(Log::INFO_LEVEL, "%s\tAuto: move to Auto Zone started\n", Utils::getCurrentTime());
		mobility->setControlMode(CANSpeedController::kSpeed);
		++current_step;
		break;
	case 2:
		//163 inches is the distance from the wall to the autozone
		if (mobility->getUltrasonicDistance() < 163 && mobility->getYEncoderDistance() < 140 && !timer->HasPeriodPassed(0.75)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Stop();
			++current_step;
		}
		break;
	case 3:
		log->write(Log::INFO_LEVEL, "%s\tAuto: move to Auto Zone ended\n", Utils::getCurrentTime());
		++current_step;
		break;
	default:
		break;
	}
}

void Autonomous::stackTote() {
	// picks up one tote, moves it to the landmark, and stacks it
	switch (current_step) {
	case 1:
		log->write(Log::INFO_LEVEL, "%s\tAuto: stack tote started\n", Utils::getCurrentTime());
		++current_step;
		break;
	case 2:
		// moving to the tote
		// assumes the robot is at a 90 degree angle to the landmark (facing tote 1 from the right)
		manipulator->raiseFlaps(true);
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(FIRST_TOTE_TIMEOUT)) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			log->write(Log::INFO_LEVEL, "%s\tMoved to tote\n", Utils::getCurrentTime());
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 3:
		// picking up the tote
		if (!timer->HasPeriodPassed(PULL_TIMEOUT)) {
			manipulator->pullTote();
		}
		else {
			log->write(Log::INFO_LEVEL, "%s\tAuto: Pulled tote in\n", Utils::getCurrentTime());
			manipulator->raiseFlaps(false);
			manipulator->setTargetLevel(2);
			timer->Reset();
			// turn the robot so that it is facing the alliance wall
			mobility->setRotationDegrees(90);
			++current_step;
		}
		break;
	case 4:
		if (!mobility->getRotatingDegrees()) {
			log->write(Log::INFO_LEVEL, "%s\tAuto: turned 90 degrees", Utils::getCurrentTime());
			mobility->setRotationSpeed(0.0);
			timer->Reset();
			++current_step;
		}
		break;
	case 5:
		// moving to autozone
		if (mobility->getUltrasonicDistance() < 187 && mobility->getYEncoderDistance() > -140 && !timer->HasPeriodPassed(MOVE_TO_ZONE_TIMEOUT)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			log->write(Log::INFO_LEVEL, "%s\tAuto: brought tote to autozone\n", Utils::getCurrentTime());
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 6:
		/*
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
		 */
		++current_step;
		break;
	case 7:
		/*
		 // moving to landmark
		 switch (starting_location) {
		 case FAR_LEFT:
		 case FAR_RIGHT:
		 //move forward
		 if (mobility->getUltrasonicDistance() < 160 && mobility->getYEncoderDistance() > -53 && !timer->HasPeriodPassed(53.0 / INCHES_PER_SECOND)) {
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
		 */
		++current_step;
		break;
	case 8:
		/*
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
		 */
		++current_step;
		break;
	case 9:
		/*
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
		 */
		++current_step;
		break;
	case 10:
		// place the tote
		manipulator->raiseFlaps(true);
		manipulator->pushTote();
		// wait a brief moment to be sure that we've actually placed the tote
		if (timer->HasPeriodPassed(PUSH_TIMEOUT)) {
			log->write(Log::INFO_LEVEL, "%s\tAuto: placed tote in autozone\n", Utils::getCurrentTime());
			++current_step;
			timer->Reset();
		}
		break;
	case 11:
		// back up
		if (timer->HasPeriodPassed(0.5)) {
			log->write(Log::INFO_LEVEL, "%s\tBacked up from tote\n", Utils::getCurrentTime());
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		else {
			mobility->setDirection(0.0, 0.75);
		}
		break;
	case 12:
		log->write(Log::INFO_LEVEL, "%s\tAuto: stack tote ended\n", Utils::getCurrentTime());
		++current_step;
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
		manipulator->raiseFlaps(true);
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(0.3)) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			log->write(Log::INFO_LEVEL, "%s\tMoved to container\n", Utils::getCurrentTime());
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 2:
		// picking up the container
		manipulator->pullTote();
		if (timer->HasPeriodPassed(PULL_TIMEOUT)) {
			log->write(Log::INFO_LEVEL, "%s\tPicked up container\n", Utils::getCurrentTime());
			manipulator->raiseFlaps(false);
			manipulator->setTargetLevel(1);
			// turn to face the alliance wall
			mobility->setRotationDegrees(90);
			timer->Reset();
			++current_step;
		}
		break;
	case 3:
		// move into the auto zone
		if (mobility->getUltrasonicDistance() < 187 && mobility->getYEncoderDistance() > -140 && !timer->HasPeriodPassed(MOVE_TO_ZONE_TIMEOUT)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			log->write(Log::INFO_LEVEL, "%s\tMoved into the autozone with a container\n", Utils::getCurrentTime());
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 4:
		// put down the container
		manipulator->setTargetLevel(0);
		if (manipulator->getLevel() == 0) {
			manipulator->raiseFlaps(true);
			manipulator->pushTote();
			++current_step;
		}
		break;
	case 5:
		// backing up to be sure we aren't touching the container
		if (mobility->getUltrasonicDistance() < 14 && mobility->getYEncoderDistance() > -16 && !timer->HasPeriodPassed(0.4)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			log->write(Log::INFO_LEVEL, "%s\tPlaced container and backed up\n", Utils::getCurrentTime());
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
	switch(current_step) {
	case 1:
		if(mobility->getYEncoderDistance() >= 24) {
			mobility->setDirection(0.0,0.0);
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		else {
			mobility->setDirection(0.0, 0.25);
		}
	}

//assuming we're facing container, we pick up container, place it on tote, then move into autozone
	/*switch (current_step) {
	case 1:
		// moving to the container
		// assumes the robot is at a +90 degree angle to the landmark (facing container 1 from the left)
		//sam chanesman
		manipulator->raiseFlaps(true);
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(FIRST_TOTE_TIMEOUT)) {
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
		// picking up the container
		manipulator->pullTote();
		if (timer->HasPeriodPassed(PULL_TIMEOUT)) {
			manipulator->raiseFlaps(false);
			manipulator->setTargetLevel(1);
			mobility->setRotationDegrees(90);
			timer->Reset();
			++current_step;
		}
		break;
	case 3:
		// moving to the tote
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 18 && !timer->HasPeriodPassed(FIRST_TOTE_TIMEOUT)) {
			mobility->setDirection(0.0, -0.75);
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
			manipulator->raiseFlaps(true);
			manipulator->pushTote();	//pushing container, not tote, fyi
			++current_step;
		}
		break;
	case 5:
		// back up + lower manipulator lift
		manipulator->setTargetLevel(0);
		// one foot is an arbitrary distance to back up
		if (mobility->getUltrasonicDistance() < 12 && mobility->getYEncoderDistance() > -14 && !timer->HasPeriodPassed(BACKUP_TIMEOUT)) {
			mobility->setDirection(0.0, 0.75);
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
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 14 && !timer->HasPeriodPassed(BACKUP_TIMEOUT)) {
			mobility->setDirection(0.0, -0.75);
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
		if (timer->HasPeriodPassed(PULL_TIMEOUT)) {
			manipulator->raiseFlaps(false);
			manipulator->setTargetLevel(1);
			timer->Reset();
			++current_step;
		}
		break;
	case 8:
		//move into auto zone
		if (mobility->getUltrasonicDistance() < 187 && mobility->getYEncoderDistance() > -140 && !timer->HasPeriodPassed(MOVE_TO_ZONE_TIMEOUT)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 9:
		//put it down
		manipulator->setTargetLevel(0);
		if (manipulator->getLevel() == 0) {
			manipulator->raiseFlaps(true);
			manipulator->pushTote();
			timer->Reset();
			++current_step;
		}
		break;
	case 10:
		//back up so we're sure we aren't touching the tote/container
		if (mobility->getUltrasonicDistance() < 14 && mobility->getYEncoderDistance() > -14 && !timer->HasPeriodPassed(0.4)) {
			mobility->setDirection(0.0, 0.75);
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
	}*/
}

void Autonomous::centerContainers() {
//facing wall, starting in between auto zone and landfill zone, move to pick up containers on step
	switch (current_step) {
	case 1:

		if (timer->HasPeriodPassed(1.5)) {
			log->write(Log::INFO_LEVEL, "%s\tAuto: lowered rakes\n", Utils::getCurrentTime());
			manipulator->movePortRake(Manipulator::RAKE_STILL);
			manipulator->moveStarboardRake(Manipulator::RAKE_STILL);
			timer->Reset();
			++current_step;
		}
		else {
			manipulator->movePortRake(Manipulator::RAKE_LOWERING);
			manipulator->moveStarboardRake(Manipulator::RAKE_LOWERING);
		}
		break;
	case 2:
		// move backwards towards step
		// 240 is a guess - we need to move back until the back of the robot is at the landfill
		if (mobility->getUltrasonicDistance() < 240 && mobility->getYEncoderDistance() > -24 && !timer->HasPeriodPassed(0.5)) { //distance to alliance wall
			mobility->setDirection(0.0, 0.75);
		}
		else {
			log->write(Log::INFO_LEVEL, "%s\tMoved to landfill zone\n", Utils::getCurrentTime());
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 3:
		// extend rakes and hook the containers
		manipulator->movePortRake(Manipulator::RAKE_LIFTING);
		manipulator->moveStarboardRake(Manipulator::RAKE_LIFTING);
		// wait a moment to allow the rakes to do their thing. not sure how long this will take
		if (timer->HasPeriodPassed(1.0)) {
			log->write(Log::INFO_LEVEL, "%s\tExtended rakes to grab center containers\n", Utils::getCurrentTime());
			manipulator->movePortRake(Manipulator::RAKE_STILL);
			manipulator->moveStarboardRake(Manipulator::RAKE_STILL);
			timer->Reset();
			++current_step;
		}
		break;
	case 4:
		//move forward again to pull the containers off, and to get ourselves into the autozone
		if (mobility->getUltrasonicDistance() > 190 && mobility->getYEncoderDistance() < 60 && !timer->HasPeriodPassed(MOVE_TO_ZONE_TIMEOUT)) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			log->write(Log::INFO_LEVEL, "%s\tMoved into autozone with center containers\n", Utils::getCurrentTime());
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 5:
		//yay done :D
		break;
	}
}

void Autonomous::moveTwoTotes() {
// move two tote stack into the auto zone on landmark
	switch (current_step) {
	case 1:
		// scoot in towards tote
		if (mobility->getUltrasonicDistance() > 3 && mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(FIRST_TOTE_TIMEOUT)) {
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
		// picking up the tote
		// wait to ensure that the tote has actually been pulled in
		if (timer->HasPeriodPassed(PULL_TIMEOUT)) {
			timer->Reset();
			manipulator->raiseFlaps(false);
			manipulator->setTargetLevel(1);
			++current_step;
		}
		else {
			manipulator->pullTote();
		}
		break;
	case 3:
		// navigate the container - we move infield to move around it
		if (mobility->getUltrasonicDistance() < 30 && mobility->getXEncoderDistance() < 36 && !timer->HasPeriodPassed(0.7)) {
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
			mobility->setDirection(0.0, -0.75);
		}
		else if (mobility->getUltrasonicDistance() < 184.0) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			++current_step;
		}
		break;
	case 5:
		// scoot in sideways so that we can grab the tote
		if (mobility->getUltrasonicDistance() > 15 && mobility->getXEncoderDistance() > -36 && !timer->HasPeriodPassed(0.7)) {
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
		if (mobility->getUltrasonicDistance() > 3 && fabs(mobility->getYEncoderDistance()) < 10 && !timer->HasPeriodPassed(0.3)) {
			mobility->setDirection(0.0, -0.75);
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
		manipulator->raiseFlaps(true);
		manipulator->pushTote();
		// wait briefly to make sure we actually pushed the tote out
		if (timer->HasPeriodPassed(PUSH_TIMEOUT)) {
			timer->Reset();
			++current_step;
		}
		break;
	case 8:
		// back up + lower manipulator lift
		manipulator->setTargetLevel(0);
		// one foot is an arbitrary distance to back up
		if (mobility->getUltrasonicDistance() < 12 && fabs(mobility->getYEncoderDistance()) < 12 && !timer->HasPeriodPassed(BACKUP_TIMEOUT)) {
			mobility->setDirection(0.0, 0.75);
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
		if (mobility->getUltrasonicDistance() > 3 && fabs(mobility->getYEncoderDistance()) < 12 && !timer->HasPeriodPassed(BACKUP_TIMEOUT)) {
			mobility->setDirection(0.0, -0.75);
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
		if (timer->HasPeriodPassed(PULL_TIMEOUT)) {
			timer->Reset();
			manipulator->raiseFlaps(false);
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
		if (mobility->getUltrasonicDistance() < 148 && fabs(mobility->getYEncoderDistance()) < 100 && !timer->HasPeriodPassed(MOVE_TO_ZONE_TIMEOUT)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		break;
	case 12:
		/*
		 //turn to face landmark
		 mobility->setRotationDegrees(180);
		 //reset/start a timer so that we can move forward briefly to get very close to the landmark
		 timer->Reset();
		 */
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
		/*
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
		 */
		timer->Reset();
		++current_step;
		break;
	case 14:
		// place the tote
		manipulator->raiseFlaps(true);
		manipulator->pushTote();
		if (timer->HasPeriodPassed(PUSH_TIMEOUT)) {
			++current_step;
			timer->Reset();
		}
		break;
	case 15:
		// back up
		if (timer->HasPeriodPassed(0.5)) {
			mobility->setDirection(0.0, 0.0);
			timer->Stop();
			++current_step;
		}
		else {
			mobility->setDirection(0.0, 0.75);
		}
		break;
	case 16:
		// yay we're done
		break;
	}

}

void Autonomous::moveThreeTotes() {
	const float pull_tote_time = 0.1;
	const float close_flaps_time = 0.2;
	const float move_right_time = 1.3;
	const float wait_for_stop_time = 0.1;
	const float forward_past_container_time = 1.5;
	const float move_left_time = 1.3;
	const float forward_into_tote_time = 0.5;
	const float rotate_right_zone_time = 0.6;
	const float forward_into_zone_time = 1.5;
	const float open_flaps_time = 0.1;
	const float rotate_right_time = 0.1;

	const float right_distance = 26.0;
	const float forward_past_container_distance = 62.0;
	const float left_distance = 26.0;
	const float forward_into_tote_distance = 16.0;
	const float forward_into_zone_distance = 100.0;

	const float right_speed = 0.3;
	const float left_speed = -0.3;
	const float forward_container_speed = 0.3;
	const float forward_tote_speed = 0.2;
	const float forward_zone_speed = 0.3;
	const float rotate_right_speed = 0.5;


	switch(current_step) {
	case 1:
		log->write(Log::INFO_LEVEL, "%s\tStarting move three totes play\n", Utils::getCurrentTime());
		mobility->setControlMode(CANTalon::kSpeed);
		mobility->setDirection(0.0, 0.0);
		mobility->setRotationSpeed(0.0);
		mobility->resetXEncoderDistance();
		mobility->resetYEncoderDistance();
		mobility->rotClosedLoop(true);
		mobility->setRotationDegrees(0.0);
		++current_step;
		break;
	case 2:
		// go down the line of totes, picking up each one and pushing aside the containers
		// then drive sideways into the auto zone, put down the stack, and back away enough that we aren't touching it
		// start with arms already surrounding first tote

		// picking up the tote
		// wait to ensure that the tote has actually been pulled in
		if (timer->HasPeriodPassed(pull_tote_time)) {
			manipulator->moveTote(0.0,0.0);
			timer->Reset();
			++current_step;
		}
		else {
			manipulator->moveTote(-1.0,0.0);//TODO:Check if forwards is inverted
		}
		break;
	case 3:
		//Close flaps
		log->write(Log::INFO_LEVEL, "%s\tClosing flaps\n", Utils::getCurrentTime());
		if(timer->HasPeriodPassed(close_flaps_time)) {
			manipulator->moveFlaps(Manipulator::FLAP_STILL);
			timer->Reset();
			++current_step;
		}
		else {
			manipulator->moveFlaps(Manipulator::FLAP_LOWERING);
		}
		break;
	case 4:
		//Raise lifter
		log->write(Log::INFO_LEVEL, "%s\tRaising Lifter\n", Utils::getCurrentTime());
		if(manipulator->getLevel() == 1) {
			timer->Reset();
			++current_step;
		}
		else {
			manipulator->setTargetLevel(1);
		}
		break;
	case 5:
		//Move Right, around container
		log->write(Log::INFO_LEVEL, "%s\tMoving right around container 1\n", Utils::getCurrentTime());
		if((fabs(mobility->getXEncoderDistance()) >= right_distance) || (timer->HasPeriodPassed(move_right_time))) {
			mobility->setDirection(0.0,0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		else {
			mobility->setDirection(right_speed, 0.0);
		}
		break;
	case 6:
		//Wait for robot to stop
		log->write(Log::INFO_LEVEL, "%s\tWaiting for robot to  stop\n", Utils::getCurrentTime());
		if(mobility->isVelZero() || timer->HasPeriodPassed(wait_for_stop_time)) {
			timer->Reset();
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setDirection(0.0,0.0);
		}
		break;
	case 7:
		//Move forward past container
		log->write(Log::INFO_LEVEL, "%s\tMoving forward past container 1\n", Utils::getCurrentTime());
		if((fabs(mobility->getYEncoderDistance()) >= forward_past_container_distance) ||
				(timer->HasPeriodPassed(forward_past_container_time))) {
			mobility->setDirection(0.0, 0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		else {
			mobility->setDirection(0.0, forward_container_speed);
		}
		break;
	case 8:
		//Wait for robot to stop
		log->write(Log::INFO_LEVEL, "%s\tWaiting for robot to  stop\n", Utils::getCurrentTime());
		if(mobility->isVelZero() || timer->HasPeriodPassed(wait_for_stop_time)) {
			timer->Reset();
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setDirection(0.0,0.0);
		}
		break;
	case 9:
		//Move left infront of tote 2
		log->write(Log::INFO_LEVEL, "%s\tMoving left infront of tote 2\n", Utils::getCurrentTime());
		if((fabs(mobility->getXEncoderDistance()) >= left_distance) || (timer->HasPeriodPassed(move_left_time))) {
			mobility->setDirection(0.0, 0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		else {
			mobility->setDirection(left_speed, 0.0);
		}
		break;
	case 10:
		//Wait for robot to stop
		log->write(Log::INFO_LEVEL, "%s\tWaiting for robot to  stop\n", Utils::getCurrentTime());
		if(mobility->isVelZero() || timer->HasPeriodPassed(wait_for_stop_time)) {
			timer->Reset();
			mobility->setRotationSpeed(0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setDirection(0.0,0.0);
			mobility->setRotationSpeed(rotate_right_speed);
		}
		break;
	case 11:
		//Move forward into tote 2
		log->write(Log::INFO_LEVEL, "%s\tMoving into tote 2\n", Utils::getCurrentTime());
		if((fabs(mobility->getYEncoderDistance()) >= forward_into_tote_distance) ||
				(timer->HasPeriodPassed(forward_into_tote_time))) {
			mobility->setDirection(0.0, 0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		else {
			mobility->setDirection(0.0, forward_tote_speed);
			//TODO: If we get a second set of intake wheels, run them here
		}
		break;
	case 12:
		//Wait for robot to stop
		log->write(Log::INFO_LEVEL, "%s\tWaiting for robot to  stop\n", Utils::getCurrentTime());
		if(mobility->isVelZero() || timer->HasPeriodPassed(wait_for_stop_time)) {
			timer->Reset();
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setDirection(0.0,0.0);
		}
		break;
	case 13:
		//Lower lifter
		log->write(Log::INFO_LEVEL, "%s\tLowering lifter onto tote 2\n", Utils::getCurrentTime());
		if(manipulator->getLevel() == 0) {
			timer->Reset();
			++current_step;
		}
		else {
			manipulator->setTargetLevel(0);
		}
		break;
	case 14:
		//Pull in tote
		log->write(Log::INFO_LEVEL, "%s\tPulling in tote 2\n", Utils::getCurrentTime());
		if(timer->HasPeriodPassed(pull_tote_time)) {
			timer->Reset();
			manipulator->moveTote(0.0,0.0);
			++current_step;
		}
		else {
			manipulator->moveTote(-1.0,0.0);
		}
		break;
	case 15:
		//Raise totes
		log->write(Log::INFO_LEVEL, "%s\tLifting tote 2\n", Utils::getCurrentTime());
		if(manipulator->getLevel() == 1) {
			timer->Reset();
			++current_step;
		}
		else {
			manipulator->setTargetLevel(1);
		}
		break;
	case 16:
		//Move right around container
		log->write(Log::INFO_LEVEL, "%s\tMoving right around container 2\n", Utils::getCurrentTime());
		if((fabs(mobility->getXEncoderDistance()) >= right_distance) || (timer->HasPeriodPassed(move_right_time))) {
			mobility->setDirection(0.0,0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		else {
			mobility->setDirection(right_speed, 0.0);
		}
		break;
	case 17:
		//Wait for robot to stop
		log->write(Log::INFO_LEVEL, "%s\tWaiting for robot to  stop\n", Utils::getCurrentTime());
		if(mobility->isVelZero() || timer->HasPeriodPassed(wait_for_stop_time)) {
			timer->Reset();
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setDirection(0.0,0.0);
		}
		break;
	case 18:
		//Move forward past container
		log->write(Log::INFO_LEVEL, "%s\tMoving forward past container 2\n", Utils::getCurrentTime());
		if((fabs(mobility->getYEncoderDistance()) >= forward_past_container_distance) ||
				(timer->HasPeriodPassed(forward_past_container_time))) {
			mobility->setDirection(0.0, 0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		else {
			mobility->setDirection(0.0, forward_container_speed);
		}
		break;
	case 19:
		//Wait for robot to stop
		log->write(Log::INFO_LEVEL, "%s\tWaiting for robot to  stop\n", Utils::getCurrentTime());
		if(mobility->isVelZero() || timer->HasPeriodPassed(wait_for_stop_time)) {
			timer->Reset();
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setDirection(0.0,0.0);
		}
		break;
	case 20:
		//Move left infront of tote 3
		log->write(Log::INFO_LEVEL, "%s\tMoving left infront of tote 3\n", Utils::getCurrentTime());
		if((fabs(mobility->getXEncoderDistance()) >= left_distance) || (timer->HasPeriodPassed(move_left_time))) {
			mobility->setDirection(0.0, 0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		else {
			mobility->setDirection(left_speed, 0.0);
		}
		break;
	case 21:
		//Wait for robot to stop
		log->write(Log::INFO_LEVEL, "%s\tWaiting for robot to  stop\n", Utils::getCurrentTime());
		if(mobility->isVelZero() || timer->HasPeriodPassed(wait_for_stop_time)) {
			timer->Reset();
			mobility->setRotationSpeed(0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setDirection(0.0,0.0);
			mobility->setRotationSpeed(rotate_right_speed);
		}
		break;
	case 22:
		//Move forward into tote 3
		log->write(Log::INFO_LEVEL, "%s\tMoving into tote 3\n", Utils::getCurrentTime());
		if((fabs(mobility->getYEncoderDistance()) >= forward_into_tote_distance) ||
				(timer->HasPeriodPassed(forward_into_tote_time))) {
			mobility->setDirection(0.0, 0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			timer->Reset();
			++current_step;
		}
		else {
			mobility->setDirection(0.0, forward_tote_speed);
			//TODO: If we get a second set of intake wheels, run them here
		}
		break;
	case 23:
		//Wait for robot to stop
		log->write(Log::INFO_LEVEL, "%s\tWaiting for robot to  stop\n", Utils::getCurrentTime());
		if(mobility->isVelZero() || timer->HasPeriodPassed(wait_for_stop_time)) {
			timer->Reset();
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setDirection(0.0,0.0);
		}
		break;
	case 24:
		//Lower lifter
		log->write(Log::INFO_LEVEL, "%s\tLowering lifter onto tote 3\n", Utils::getCurrentTime());
		if(manipulator->getLevel() == 0) {
			timer->Reset();
			++current_step;
		}
		else {
			manipulator->setTargetLevel(0);
		}
		break;
	case 25:
		//Pull in tote
		log->write(Log::INFO_LEVEL, "%s\tPulling in tote 3\n", Utils::getCurrentTime());
		if(timer->HasPeriodPassed(pull_tote_time)) {
			timer->Reset();
			manipulator->moveTote(0.0,0.0);
			++current_step;
		}
		else {
			manipulator->moveTote(-1.0,0.0);
		}
		break;
	case 26:
		//Rotate right towards zone
		log->write(Log::INFO_LEVEL, "%s\tRotating to face auto zone\n", Utils::getCurrentTime());
		if(timer->HasPeriodPassed(rotate_right_zone_time)) {
			mobility->setRotationSpeed(0.0);
			timer->Reset();
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setRotationSpeed(0.5);
		}
		break;
	case 27:
		//Wait for robot to stop
		log->write(Log::INFO_LEVEL, "%s\tWaiting for robot to  stop\n", Utils::getCurrentTime());
		if(mobility->isVelZero() ||timer->HasPeriodPassed(wait_for_stop_time)) {
			timer->Reset();
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		else {
			mobility->setDirection(0.0,0.0);
			mobility->setRotationSpeed(0.0);
		}
		break;
	case 28:
		//Forward into zone
		log->write(Log::INFO_LEVEL, "%s\tMoving forward into auto zone\n", Utils::getCurrentTime());
		if(fabs(mobility->getYEncoderDistance()) >= forward_into_zone_distance ||
				timer->HasPeriodPassed(forward_into_zone_time)) {
			++current_step;
			timer->Reset();
			mobility->setDirection(0.0,0.0);
			mobility->resetXEncoderDistance();
			mobility->resetYEncoderDistance();
		}
		else {
			mobility->setDirection(0.0, forward_zone_speed);
		}
		break;
	case 29:
		//Open flaps
		log->write(Log::INFO_LEVEL, "%s\tOpenning flaps\n", Utils::getCurrentTime());
		if(timer->HasPeriodPassed(open_flaps_time)) {
			manipulator->moveFlaps(Manipulator::FLAP_STILL);
			++current_step;
		}
		else {
			manipulator->moveFlaps(Manipulator::FLAP_RAISING);
		}
		break;
	default:
		break;

/*	case 2:
		// move forward to the container
		if (mobility->getYEncoderDistance() < 12 || !timer->HasPeriodPassed(0.7)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 3:
		// move slightly to the side in preparation for smacking the container aside
		if (!timer->HasPeriodPassed(0.2)) {
			mobility->setRotationSpeed(0.5);
			mobility->setDirection(0.5, 0.75);
		}
		else {
			timer->Reset();
			mobility->setRotationSpeed(0.0);
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 4:
		// turn back
		if (!timer->HasPeriodPassed(0.2)) {
			mobility->setRotationSpeed(-0.5);
			mobility->setDirection(-0.5, 0.75);
		}
		else {
			timer->Reset();
			mobility->setRotationSpeed(0.0);
			mobility->setDirection(0.0, 0.0);
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 5:
		// move forward to the next tote
		if ((mobility->getUltrasonicDistance() > 2 && mobility->getYEncoderDistance() < 30) || !timer->HasPeriodPassed(1.5)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			timer->Reset();
			mobility->setDirection(0.0, 0.0);
		}
		break;
	case 6:
		// place the first tote on top of the second
		manipulator->raiseFlaps(true);
		manipulator->pushTote();
		// wait briefly to make sure we actually pushed the tote out
		if (timer->HasPeriodPassed(0.5)) {
			timer->Reset();
			++current_step;
		}
		break;
	case 7:
		// lower lifter around the bottom tote
		manipulator->setTargetLevel(0);

		if (timer->HasPeriodPassed(0.3) || manipulator->getLevel() == 0) {
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 8:
		//scoot forward again
		if (mobility->getUltrasonicDistance() > 3 && fabs(mobility->getYEncoderDistance()) < 12 && !timer->HasPeriodPassed(1.5)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 9:
		// pick up the two stacked totes and start lifting them
		// wait to ensure that the totes have actually been pulled in
		if (!timer->HasPeriodPassed(0.5)) {
			manipulator->pullTote();
		}
		else {
			timer->Reset();
			manipulator->raiseFlaps(false);
			manipulator->setTargetLevel(2);
			++current_step;
		}
		break;
	case 10:
		// move forward to the container
		if (mobility->getYEncoderDistance() < 12 && !timer->HasPeriodPassed(0.7)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 11:
		// move slightly to the side in preparation for smacking the container aside
		if (!timer->HasPeriodPassed(0.2)) {
			mobility->setRotationSpeed(0.5);
			mobility->setDirection(0.5, 0.75);
		}
		else {
			timer->Reset();
			mobility->setRotationSpeed(0.0);
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 12:
		// turn back
		if (!timer->HasPeriodPassed(0.2)) {
			mobility->setRotationSpeed(-0.5);
			mobility->setDirection(-0.5, 0.75);
		}
		else {
			timer->Reset();
			mobility->setRotationSpeed(0.0);
			mobility->setDirection(0.0, 0.0);
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 13:
		// move forward to the last tote
		if ((mobility->getUltrasonicDistance() > 2 && mobility->getYEncoderDistance() < 30) || !timer->HasPeriodPassed(1.5)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			timer->Reset();
			mobility->setDirection(0.0, 0.0);
		}
		break;
	case 14:
		// place the totes on top of the third
		manipulator->raiseFlaps(true);
		manipulator->pushTote();
		// wait briefly to make sure we actually pushed the totes out
		if (timer->HasPeriodPassed(0.4)) {
			timer->Reset();
			++current_step;
		}
		break;
	case 15:
		// lower manipulator lift around bottom tote
		manipulator->setTargetLevel(0);

		if (manipulator->getLevel() == 0 || timer->HasPeriodPassed(0.3)) {
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 16:
		//scoot forward again
		if ((mobility->getUltrasonicDistance() > 3 && fabs(mobility->getYEncoderDistance())) < 12 || !timer->HasPeriodPassed(0.4)) {
			mobility->setDirection(0.0, 0.75);
		}
		else {
			mobility->setDirection(0.0, 0.0);
			timer->Reset();
			mobility->resetYEncoderDistance();
			++current_step;
		}
		break;
	case 17:
		// pick up the three stacked totes and start lifting them
		// wait to ensure that the totes have actually been pulled in
		if (!timer->HasPeriodPassed(0.4)) {
			manipulator->pullTote();
		}
		else {
			timer->Reset();
			manipulator->raiseFlaps(false);
			manipulator->setTargetLevel(1);
			mobility->resetXEncoderDistance();
			++current_step;
		}
		break;
	case 18:
		// move into the auto zone
		if (mobility->getXEncoderDistance() < 108 && !timer->HasPeriodPassed(MOVE_TO_ZONE_TIMEOUT)) {
			mobility->setDirection(0.75, 0.0);
		}
		else {
			timer->Reset();
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 19:
		// put down the totes
		manipulator->raiseFlaps(true);
		manipulator->pushTote();
		if (timer->HasPeriodPassed(0.4)) {
			timer->Reset();
			++current_step;
		}
		break;
	case 20:
		// back up
		if (!timer->HasPeriodPassed(0.7)) {
			mobility->setDirection(0.0, -0.75);
		}
		else {
			timer->Stop();
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	case 21:
		// yay we're done
		break;*/
	}
}
