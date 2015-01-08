#ifndef MOBILITY_H_
#define MOBILITY_H_

#include "WPILib.h"

class Mobility
{
public:
	static Mobility* getInstance();

private:
	Mobility();
	static Mobility* INSTANCE;

};

#endif // MOBILITY_H_