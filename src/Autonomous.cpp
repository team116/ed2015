#include <WPILib.h>
#include "Ports.h"
#include "Autonomous.h"
#include "Mobility.h"
#include "Log.h"
#include "Manipulator.h"


Autonomous* Autonomous::INSTANCE = NULL;

Autonomous::Autonomous(int delay, int play, int location)
{
	current_step = 1;
	starting_location = location;
	this->play = play;

	delay_timer = new Timer();
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();
	log = Log::getInstance();
}

void Autonomous::process()
{
	switch (play)
	{
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

void Autonomous::doNothing()
{
	// nada
}

void Autonomous::moveToZone()
{
	switch (current_step)
	{
	case 1:
		if (mobility->getUltrasonicDistance() < 163) // distance to landmark in inches
		{
			mobility->setDirection(0.0, 0.75);
		}
		else
		{
			mobility->setDirection(0.0, 0.0);
			++current_step;
		}
		break;
	default:
		break;
	}
}

void Autonomous::stackTote()
{
	switch (current_step)
	{

	case 1:
		// assumes the robot is at a -90 degree angle to the landmark (facing tote 1 from the right)
		manipulator->setFlaps(false);
		if(mobility->getUltrasonicDistance() < 35){
			mobility->setDirection(0.0, 0.5);
		}
		else if(mobility->getUltrasonicDistance() > 3){
			mobility->setDirection(0.0, 0.0);
		}
		manipulator->setFlaps(true);
		// turn the robot so that it is facing the allaince wall
		if(mobility < 124){
			mobility->setDirection(0.0, -0.5);

		}


	}
	case 2:

}

void Autonomous::moveContainer()
{
	switch (current_step)
	{

	}
}

void Autonomous::moveContainerAndTote()
{
	switch (current_step)
	{

	}
}

Autonomous* Autonomous::getInstance(int delay, int play, int location)
{
    if (INSTANCE == NULL) {
        INSTANCE = new Autonomous(delay, play, location);
    }
    return INSTANCE;
}
