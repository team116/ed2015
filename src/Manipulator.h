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
	void grab();
	void closeArms(bool close);
	void moveToHeight(int level); //move to one of preset levels
	void changeHeight(float change); // to move up/down depending on positive/negative by change # of inches
	int getLevel();
	float getHeight(); // to return height in inches
	void startConveyorBelt();
	void stopConveyorBelt();
    void honor_limits(bool to_use_or_not_to_use);
	void read_limits();
	void liftLifters();
	void liftRakes();
	void spinWheels();


private:
	static Manipulator* INSTANCE;
	CANTalon* left_wheel;
	CANTalon* right_wheel;
	CANTalon* lifter_one;
	CANTalon* lifter_two;
	CANTalon* rake_port;
	CANTalon* rake_starboard;
	CANTalon* close_arms;


	float current_height; //inches
	float target_height;
	bool belt_moving;
	bool using_limits;

	//motors n' stuff need to go here
};


#endif /* SRC_MANIPULATOR_H_ */
