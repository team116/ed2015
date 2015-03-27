#ifndef SRC_MANIPULATOR_H_
#define SRC_MANIPULATOR_H_

#include <WPILib.h>
#include <CANTalon.h>
#include <Timer.h>
#include "Mobility.h"
#include "Log.h"
#include <ctime>
#include <Servo.h>
#include <RobotDrive.h>

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
		FLAP_LOWERING,
		FLAP_RAISING,
		FLAP_STILL
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

	void moveTote(float forwards, float rotate);
	void pullTote();
	void pushTote();
	void spinTote(float direction);

	void raiseFlaps(bool close);
	void moveFlaps(flap_directions dir);

	void setSurface(float s);
	void setTargetLevel(int level);
	void setFlapPosition(float p);	//see flap_position enum in private section
	void setRakePosition(rake_positions p);	//see rake_position enum in private section
	void liftLifters(lifter_directions direction);
	void setLifterModifier(float power);
	int getLevel();
	float getHeight();
	float getVoltageCount();
	int getFlapAngle();
	void usingEncoder(bool enc);
	void useLifterWheels(bool lifter);
	bool usingLifterWheels();

	bool isLimitReached();

	void liftRakes(bool going_up);
	void movePortRake(rake_directions direction);
	void moveStarboardRake(rake_directions direction);


	void honorLimits(bool to_use_or_not_to_use);

	void moveTrexArms(servos_position trex_arm_position);
	void moveRakeStabilizers(servos_position trex_arm_position);
	void moveContainerBlocker(servos_position container_block_position);


	static const float FLOOR;
	static const float SCORING_PLATFORM;
	static const float STEP;

	// stores potentiometer values for the diff. positions for the flaps
	static const int FLAP_ANGLE_LOW;	//closed
	static const int FLAP_ANGLE_MID;
	static const int FLAP_ANGLE_HIGH;	//opened

private:
	static Manipulator* INSTANCE;
	// motors
	CANTalon* left_lifter_wheel;
	CANTalon* right_lifter_wheel;
	CANTalon* left_base_wheel;
	CANTalon* right_base_wheel;
	CANTalon* lifter_one;
	//CANTalon* lifter_two;
	CANTalon* rake_port;
	CANTalon* rake_starboard;
	CANTalon* close_flaps;

	RobotDrive* lifter_tote_wheels;
	RobotDrive* base_tote_wheels;

	static const float P;
	static const float I;
	static const float D;
	static const unsigned int IZone;
	//AnalogPotentiometer* potentiometer;	//TODO: rewrite potentiometer to go through Talon
	static const int PULSE_PER_REV;
	static const float INCH_PER_REV;
	flap_directions dir_not_possible;

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

	Servo* container_blocker;
	bool can_deploy_servo;
	static const float CONTAINER_BLOCKER_OUT;
	static const float CONTAINER_BLOCKER_REFACTOR;

	// timers
	Timer* process_timer; // used to increment the lifter position;
	Timer* flap_timer;
	static const float FLAP_TIMEOUT_LOW_TO_MID;
	static const float FLAP_TIMEOUT_LOW_TO_HIGH;
	static const float FLAP_TIMEOUT_MID_TO_HIGH;
	Timer* rake_timer;
	static const float RAKE_TIMEOUT_LOW_TO_MID;
	static const float RAKE_TIMEOUT_LOW_TO_HIGH;
	static const float RAKE_TIMEOUT_MID_TO_HIGH;
	Timer* lifter_timer;
	static const float LEVEL_TIMEOUT;
	float lifter_timeout;
	float lifter_modifier;
	Timer* wheel_timer;
	static const float WHEEL_TIMEOUT;

	static const float MAX_FLAP_CURRENT;
	static const float FLAP_CURRENT_TIMEOUT;

	Timer* flaps_current_timer;

	// lifter stuff --inches for everything
	float current_height;
	float target_height;
	lifter_directions lifter_direction;

	static const float TOTE_HEIGHT;
	static const float LIFTER_RANGE;

	bool isInsignificantChange(float first, float second); // the order of the parameters doesn't matter
	bool canMoveLifter();
	bool flapMotionDone();
	bool rakeMotionDone();
	bool hittingRakeLimits();
	bool pushToteDone();
	bool pullToteDone();
	void usePID(bool use);

	rake_positions rake_pos;	//note: useless during teleop, do not attempt to use!!!
	rake_positions rake_pos_prev;
	flap_directions flap_state;
	int target_flap_pos;
	int flap_pos_start;
	bool using_lifter_wheels;
	wheel_directions wheel_state;
	static const float MAX_LIFTER_INCR_PER_SEC;


	//stores grace range (aka how close is close enough)
	static const float FLAP_RANGE;

	float surface;	//should always be equal to one of the platform constants
	float flap_position_raw; // raw flap position, in degrees maybe?

	bool using_limits;
	bool using_encoder;
	bool lifter_targeting;
	bool using_flap_potentiometer;
};


#endif /* SRC_MANIPULATOR_H_ */
