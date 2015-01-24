#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include <WPILib.h>
#include "Log.h"
#include "Manipulator.h"
#include "Mobility.h"

class Autonomous
{
public:
	static Autonomous* getInstance(int delay, int play, int location);

	void process();

private:
	Autonomous(int delay, int play, int location);
	static Autonomous* INSTANCE;

	Mobility* mobility;
	Manipulator* manipulator;
	Log* log;
	Timer* delay_timer;
	Timer* play_timer;

	void doNothing();
	void moveToZone();
	void stackTote();
	void moveContainer();
	void moveContainerAndTote();
	void centerContainers();

	enum Plays
	{
		DO_NOTHING = 0,
		INTO_ZONE = 1,
		STACK_TOTE = 2,
		MOVE_CONTAINER = 3,
		CONTAINER_AND_TOTE = 4,
		CENTER_CONTAINERS = 5
	};

	int current_step;
	int play;
	int starting_location;

};

#endif // AUTONOMOUS_H_
