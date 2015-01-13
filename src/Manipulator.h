#ifndef SRC_MANIPULATOR_H_
#define SRC_MANIPULATOR_H_

#include "WPILib.h"
#include "Arduino.h"
#include "Mobility.h"
//#include "UDPListener.h"
#include "Log.h"
//#include "Piston.h"

class Manipulator {
public:
	Manipulator();
	virtual ~Manipulator();

	static Manipulator* getInstance();
	Arduino* arduino;
	Mobility* mobility;
	//UDPListener* listener;

	void process();
	void grab();
	void closeArms();
	void moveToHeight(int level);
	int getHeight();
	void openArms();
	void startConveyorBelt();
	void stopConveyorBelt();
    void honor_limits(bool to_use_or_not_to_use);
	void read_limits();
private:
	static Manipulator* INSTANCE;

	int height;
	bool belt_moving;
	bool using_limits;

	//motors n' stuff need to go here
};


#endif /* SRC_MANIPULATOR_H_ */
