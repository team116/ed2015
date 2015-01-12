#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include "WPILib.h"

class Autonomous
{
public:
	static Autonomous* getInstance();

	void process();

private:
	Autonomous();
	static Autonomous* INSTANCE;

};

#endif // AUTONOMOUS_H_
