#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include <WPILib.h>
#include "Log.h"
#include "Manipulator.h"
#include "Mobility.h"

class Autonomous
{
public:
	Autonomous(int delay, int play, int location);

	void process();

private:
	Mobility* mobility;
	Manipulator* manipulator;
	Log* log;
	Timer* delay_timer;
	Timer* timer;

	void doNothing();
	void moveToZone();
	void stackTote();
	void moveContainer();
	void moveContainerAndTote();
	void centerContainers();
	void moveTwoTotes();

	enum Plays
	{
		DO_NOTHING = 0,
		INTO_ZONE = 1,
		STACK_TOTE = 2,
		MOVE_CONTAINER = 3,
		CONTAINER_AND_TOTE = 4,
		CENTER_CONTAINERS = 5
	};

	enum starting_locations
	{
		FAR_LEFT = 0,
		CENTER = 1,
		FAR_RIGHT = 2
	};

	int current_step;
	int play;
	int starting_location;
	int delay;
	bool delay_over;

	static const float INCHES_PER_SECOND;

};

#endif // AUTONOMOUS_H_
