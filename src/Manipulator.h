#ifndef SRC_MANIPULATOR_H_
#define SRC_MANIPULATOR_H_

#include <WPILib.h>
#include <CANTalon.h>
#include <Timer.h>
#include "Mobility.h"
#include "Log.h"
#include <ctime>
#include <Servo.h>

class Manipulator
{
public:
	Manipulator();

	static Manipulator* getInstance();
	Mobility* mobility;
	Log* log;

	enum lifter_directions
	{
		MOVING_UP,
		MOVING_DOWN,
		NOT_MOVING
	};

	enum rake_directions
	{
		RAKE_LIFTING,
		RAKE_LOWERING,
		RAKE_STILL
	};

	enum rake_positions
	{
		//note: low != absolute bottom (will not hit limit switch)
		RAKE_LOW,
		RAKE_MID,
		RAKE_HIGH
	};

	enum flap_directions
	{
		FLAP_OPENING,
		FLAP_CLOSING,
		FLAP_STILL
	};

	enum flap_positions
	{
		FLAP_LOW,
		FLAP_MID,
		FLAP_HIGH
	};

	enum wheel_directions
	{
		WHEELS_PULLING,
		WHEELS_PUSHING,
		WHEELS_STILL
	};

	enum servos_position
	{
		OUT,
		DOWN
	};

	void process();

	void pullTote();
	void pushTote();
	void spinTote(float direction);

	void closeFlaps(bool close);

	void setSurface(float s);
	void setTargetLevel(int level);
	void setFlapPosition(flap_positions p);	//see flap_position enum in private section
	void setRakePosition(rake_positions p);	//see rake_position enum in private section
	void liftLifters(lifter_directions direction);
	int getLevel();
	float getHeight();

	void liftRakes(bool going_up);
	void movePortRake(rake_directions direction);
	void moveStarboardRake(rake_directions direction);


	void honorLimits(bool to_use_or_not_to_use);

	void moveTrexArms(servos_position trex_arm_position);
	void moveRakeStabilizers(servos_position trex_arm_position);


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
	/*DigitalInput* lift_upper_limit;
	DigitalInput* lift_lower_limit;
	DigitalInput* flaps_closed_limit;
	DigitalInput* flaps_opened_limit;
	DigitalInput* port_rake_limit;
	DigitalInput* starboard_rake_limit;*/

	static const float P;
	static const float I;
	static const float D;
	static const unsigned int IZone;
	//AnalogPotentiometer* potentiometer;	//TODO: rewrite potentiometer to go through Talon
	static const int PULSE_PER_REV;
	static const float INCH_PER_REV;

	static const float ENCODER_INCREMENT;

	//servos
	Servo* left_trex_arm;
	Servo* right_trex_arm;
	static const float LEFT_TREX_DOWN;
	static const float RIGHT_TREX_DOWN;
	static const float LEFT_TREX_UP;
	static const float RIGHT_TREX_UP;

	Servo* left_rake_stabilizer;
	Servo* right_rake_stabilizer;
	static const float LEFT_RAKE_STABILIZER_DOWN;
	static const float RIGHT_RAKE_STABILIZER_DOWN;
	static const float LEFT_RAKE_STABILIZER_UP;
	static const float RIGHT_RAKE_STABILIZER_UP;

	// timers
	Timer* flap_timer;
	static const float FLAP_LOW_TO_MID_TIMEOUT;
	static const float FLAP_HIGH_TO_MID_TIMEOUT;
	static const float FLAP_LOW_TO_HIGH_TIMEOUT;
	Timer* rake_timer;
	static const float RAKE_TIMEOUT_LOW_TO_MID;
	static const float RAKE_TIMEOUT_LOW_TO_HIGH;
	static const float RAKE_TIMEOUT_MID_TO_HIGH;
	Timer* lift_timer;
	static const float LEVEL_TIMEOUT;
	float lifter_timeout;
	Timer* wheel_timer;
	static const float WHEEL_TIMEOUT;

	// lifter stuff --inches for everything
	float current_height;
	float target_height;

	static const float TOTE_HEIGHT;
	static const float LIFTER_RANGE;

	bool isInsignificantChange(float first, float second); // the order of the parameters doesn't matter
	bool canMoveLifter();
	bool flapMotionDone();
	bool rakeMotionDone();
	bool hittingRakeLimits();
	bool pushToteDone();
	bool pullToteDone();

	rake_directions port_rake_direction;
	rake_directions starboard_rake_direction;
	rake_positions rake_pos;	//note: useless during teleop, do not attempt to use!!!
	rake_positions rake_pos_prev;
	flap_directions flap_state;
	flap_positions flap_pos;
	flap_positions flap_pos_prev;
	wheel_directions wheel_state;

	// stores potentiometer values for the diff. positions for the flaps
	static const float FLAP_ANGLE_LOW;	//closed
	static const float FLAP_ANGLE_MID;
	static const float FLAP_ANGLE_HIGH;	//opened

	//stores grace range (aka how close is close enough)
	static const float FLAP_RANGE;

	float surface;	//should always be equal to one of the platform constants

	bool using_limits;
};


#endif /* SRC_MANIPULATOR_H_ */
