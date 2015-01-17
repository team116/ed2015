#ifndef SRC_MANIPULATOR_H_
#define SRC_MANIPULATOR_H_

#include <WPILib.h>
#include <CANTalon.h>
#include "Mobility.h"
//#include "UDPListener.h"
#include "Log.h"
//#include "Piston.h"

class Manipulator {
public:
	Manipulator();
	virtual ~Manipulator();

	static Manipulator* getInstance();
	Mobility* mobility;
	//UDPListener* listener;

	void process();
	void pullTote();
	void pushTote();
	void setHooks(bool close);
	void setTargetHeight(int level,bool on_step); //move to one of preset levels, with parameter for whether or not step is a factor
	void changeHeight(float change); // to move up/down depending on positive/negative by change # of inches
	int getLevel();
	float getHeight(); // to return height in inches
	void startConveyorBelt();
	void stopConveyorBelt();
    void honor_limits(bool to_use_or_not_to_use);
	void read_limits();
	void liftLifters();
	void liftRakes();
	void spinTote(float direction);//use wheels to spin tote in direction [-1.0,1.0]

private:
	static Manipulator* INSTANCE;
	CANTalon* left_wheel;
	CANTalon* right_wheel;
	CANTalon* lifter_one;
	CANTalon* lifter_two;
	CANTalon* rake_port;
	CANTalon* rake_starboard;
	CANTalon* close_hooks;


	float current_height; //inches for everything
	float target_height;
	bool belt_moving;
	bool using_limits;
	const float TOTE_HEIGHT = 12.1;

	//motors n' stuff need to go here
};


#endif /* SRC_MANIPULATOR_H_ */
