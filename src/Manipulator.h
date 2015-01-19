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
	void setFlaps(bool close);
	void setSurface(float s);
	void setTargetHeight(int level); //move to one of preset levels
	float getHeight(); // to return height in inches
	int getLevel();
	void changeHeight(float change); // to move up/down depending on positive/negative by change # of inches
	void spinTote(float direction);//use wheels to spin tote in direction in range [-1.0,1.0]
	/*void startConveyorBelt();
	void stopConveyorBelt();*/
    void honor_limits(bool to_use_or_not_to_use);
	void liftLifters();
	void liftRakes();

	static const float FLOOR;
	static const float SCORING_PLATFORM;
	static const float STEP;

private:
	static Manipulator* INSTANCE;
	CANTalon* left_wheel;
	CANTalon* right_wheel;
	CANTalon* lifter_one;
	CANTalon* lifter_two;
	CANTalon* rake_port;
	CANTalon* rake_starboard;
	CANTalon* close_flaps;

	DigitalInput* lift_upper_limit;
	DigitalInput* lift_lower_limit;
	DigitalInput* flaps_upper_limit;
	DigitalInput* flaps_lower_limit;
	DigitalInput* port_rake_limit;
	DigitalInput* starboard_rake_limit;

	Encoder* encoder;
	static const int pulse_per_rev;
	static const float inch_per_rev;

	float current_height; //inches for everything
	float target_height;
	static const float TOTE_HEIGHT;

	float surface;	//should always be equal to one of the constants below

	bool using_limits;
	//bool belt_moving;
};


#endif /* SRC_MANIPULATOR_H_ */
