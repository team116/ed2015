#ifndef SRC_MANIPULATOR_H_
#define SRC_MANIPULATOR_H_

#include <WPILib.h>
#include <CANTalon.h>
#include <Timer.h>
#include "Mobility.h"
#include "Log.h"
#include <ctime>

class Manipulator {
public:
	Manipulator();
	virtual ~Manipulator();

	static Manipulator* getInstance();
	Mobility* mobility;
	Log* log;

	enum lifter_direction
	{
		MOVING_UP,
		MOVING_DOWN,
		NOT_MOVING,
	};

	void process();

	void pullTote();
	void pushTote();
	void spinTote(float direction);

	void closeFlaps(bool close);

	void setSurface(float s);
	void setTargetLevel(int level);
	void changeHeight(float change);
	void liftLifters(lifter_direction direction);
	int getLevel();
	float getHeight();

	void liftRakes(bool going_up);

	void honorLimits(bool to_use_or_not_to_use);


	static const float FLOOR;
	static const float SCORING_PLATFORM;
	static const float STEP;

private:
	static Manipulator* INSTANCE; //																			   ,`~
	//HYDRAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAANGEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAS!!! :D  :3  (/^W^)/ `',', ~
	// motors																									 ,~	`	`
	CANTalon* left_wheel;
	CANTalon* right_wheel;
	CANTalon* lifter_one;
	CANTalon* lifter_two;
	CANTalon* rake_port;
	CANTalon* rake_starboard;
	CANTalon* close_flaps;
	//hydrangeas
	//hydrangeas
	//hydrangeas
	//hydrangeas
	//hydrangeas
	//hydrangeas
	//hydrangeas
	// sensors
	DigitalInput* lift_upper_limit;
	DigitalInput* lift_lower_limit;
	DigitalInput* flaps_closed_limit;
	DigitalInput* flaps_opened_limit;
	DigitalInput* port_rake_limit;
	DigitalInput* starboard_rake_limit;

	Encoder* encoder;
	static const int PULSE_PER_REV;
	static const float inch_per_rev;

	// timers
	Timer* flap_timer;
	static const float FLAP_TIMEOUT;
	Timer* rake_timer;
	static const float RAKE_TIMEOUT;
	Timer* lift_timer;
	static const float LEVEL_TIMEOUT;
	Timer* wheel_timer;
	static const float WHEEL_TIMEOUT;

	// lifter stuff --inches for everything
	float current_height;
	float target_height;
	float lifter_timeout;

	static const float TOTE_HEIGHT;
	static const float LIFTER_RANGE;

	bool isInsignificantChange(float first, float second); // the order of the parameters doesn't matter
	bool canMoveLifter();
	bool flapMotionDone();
	bool rakeUpMotionDone();
	bool rakeDownMotionDone();
	bool pushToteDone();
	bool pullToteDone();

	// rake stuff
	enum rake_direction
	{
		RAKE_LIFTING,
		RAKE_LOWERING,
		RAKE_STILL
	};
	rake_direction rake_direction;

	// flap stuff
	enum flap_direction
	{
		FLAP_OPENING,
		FLAP_CLOSING,
		FLAP_STILL
	};
	flap_direction flap_state;

	// wheel stuff
	enum wheel_direction
	{
		WHEELS_PULLING,
		WHEELS_PUSHING,
		WHEELS_STILL
	};
	wheel_direction wheel_state;

	float surface;	//should always be equal to one of the platform constants

	bool using_limits;
};


#endif /* SRC_MANIPULATOR_H_ */
