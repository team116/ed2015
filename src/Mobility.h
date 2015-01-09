#ifndef MOBILITY_H_
#define MOBILITY_H_

#include "WPILib.h"
#include "Gyro.h"

class Mobility
{
public:
	static Mobility* getInstance();
	void process();
	void setSpeed(float speed);
	void setDirection(int degrees);

private:
	Mobility();
	static Mobility* INSTANCE;
};

#endif // MOBILITY_H_
