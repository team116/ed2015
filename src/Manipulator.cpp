#include <WPILib.h>
#include <CANTalon.h>
#include "Manipulator.h"
#include "Ports.h"
#include <cmath>

Manipulator* Manipulator::INSTANCE = NULL;

// TODO: get actual timeouts
const float Manipulator::FLAP_TIMEOUT = 1.1;
const float Manipulator::RAKE_TIMEOUT = 1.1;
const float Manipulator::LEVEL_TIMEOUT = 1.1; // the amount of time to be given for each level of movement

// lifter stuff
const float Manipulator::LIFTER_RANGE = 0.3; // the acceptable height range for our presets
const int Manipulator::PULSE_PER_REV = 64;
const float Manipulator::inch_per_rev = 4;

const float Manipulator::TOTE_HEIGHT = 12.1;
const float Manipulator::FLOOR = 0.0;
const float Manipulator::SCORING_PLATFORM = 2.0;
const float Manipulator::STEP = 6.25;

Manipulator::Manipulator()
{
	// subsystem instance getting
	mobility = Mobility::getInstance();
	log = Log::getInstance();

	left_wheel = new CANTalon(RobotPorts::LEFT_WHEEL);
	right_wheel = new CANTalon(RobotPorts::RIGHT_WHEEL);

	// lifter initializations
	lifter_one = new CANTalon(RobotPorts::LIFTER_ONE);
	lifter_two = new CANTalon(RobotPorts::LIFTER_TWO);
	lift_upper_limit = new DigitalInput(RobotPorts::LIFT_UPPER_LIMIT);
	lift_lower_limit = new DigitalInput(RobotPorts::LIFT_LOWER_LIMIT);
	encoder = new Encoder(RobotPorts::ENCODER_A, RobotPorts::ENCODER_B);
	encoder->SetDistancePerPulse(inch_per_rev/PULSE_PER_REV);	//inches per revolution / pulses per revolution = inches per pulse
	lift_timer = new Timer();
	current_height = 0; //starting height (floor level)
	target_height = 0;
	lifter_timeout = 0.0;

	// rake initializations
	rake_port = new CANTalon(RobotPorts::RAKE_PORT_MOTOR);
	rake_starboard = new CANTalon(RobotPorts::RAKE_STARBOARD_MOTOR);
	port_rake_limit = new DigitalInput(RobotPorts::PORT_RAKE_LIMIT);
	starboard_rake_limit = new DigitalInput(RobotPorts::STARBOARD_RAKE_LIMIT);
	rake_timer = new Timer();
	rake_direction = RAKE_STILL;

	// flap initializations
	close_flaps = new CANTalon(RobotPorts::CLOSE_FLAPS_MOTOR);
	flaps_closed_limit = new DigitalInput(RobotPorts::FLAPS_UPPER_LIMIT);
	flaps_opened_limit = new DigitalInput(RobotPorts::FLAPS_LOWER_LIMIT);
	flap_timer = new Timer();
	flap_state = FLAP_STILL;

	using_limits = true;
	//belt_moving = false;
	surface = 0;
}

Manipulator::~Manipulator()
{
	// TODO Auto-generated destructor stub
}


Manipulator* Manipulator::getInstance()
{
	if (INSTANCE == NULL) {
		INSTANCE = new Manipulator();
	}
	return INSTANCE;
}


void Manipulator::process()
{
	current_height = encoder->GetDistance();	//uses data from encoder to determine current height of lift
	log->write(Log::INFO_LEVEL, "Current Height: %f \t Target Height: %f",current_height,target_height);

	if (flapMotionDone()) {
		if(flap_state == FLAP_CLOSING){
			log->write(Log::INFO_LEVEL, "flaps closed\n");
		}
		else{
			log->write(Log::INFO_LEVEL, "flaps opened\n");
		}
		close_flaps->Set(0.0);
		flap_state = FLAP_STILL;
	}


	if (isInsignificantChange(current_height, target_height)) {
		log->write(Log::INFO_LEVEL, "change insignificant: lift motors stopped\n");
		lifter_one->Set(0);
		lifter_two->Set(0);
	}
	else {
		if(canMoveLifter()) {
			if(current_height < target_height) {
				log->write(Log::INFO_LEVEL, "moving lift up\n");
				lifter_one->Set(0.5);
				lifter_two->Set(0.5);
			}
			else {
				log->write(Log::INFO_LEVEL, "moving lift down\n");
				lifter_one->Set(-0.5);
				lifter_two->Set(-0.5);
			}
		}
	}


	if (lift_lower_limit->Get()){		//reset encoder to 0 every time lift hits lower limit switch
		log->write(Log::INFO_LEVEL, "Hit bottom of lift: encoder set to 0\n");
		encoder->Reset();
	}

	if (rakeUpMotionDone()){ 	//TODO: get real times
		log->write(Log::INFO_LEVEL, "Rake finished moving up\n");
		rake_port->Set(0.0);
		rake_starboard->Set(0.0);
		rake_direction = RAKE_STILL;
	}
	else if (rakeDownMotionDone()){ 	//TODO: get real timeout period
		log->write(Log::INFO_LEVEL, "Rake finished moving down\n");
		rake_port->Set(0.0);
		rake_starboard->Set(0.0);
		rake_direction = RAKE_STILL;
	}
}

bool Manipulator::canMoveLifter()
{
	if(current_height < target_height){
		return (!lift_upper_limit->Get() || !using_limits) && !lift_timer->HasPeriodPassed(lifter_timeout);
	}
	else{
		return (!lift_lower_limit->Get() || !using_limits) && !lift_timer->HasPeriodPassed(lifter_timeout);
	}
}

bool Manipulator::flapMotionDone(){
	if (flap_state == FLAP_CLOSING) {
		return (flaps_closed_limit->Get() && using_limits) || flap_timer->HasPeriodPassed(FLAP_TIMEOUT);
	}
	else if (flap_state == FLAP_OPENING) {
		return (flaps_opened_limit->Get() && using_limits) || flap_timer->HasPeriodPassed(FLAP_TIMEOUT);
	}
	else
		return true;
}

bool Manipulator::rakeUpMotionDone(){
	return rake_direction == RAKE_LIFTING && ((port_rake_limit->Get() && using_limits) || rake_timer->HasPeriodPassed(RAKE_TIMEOUT));
}

bool Manipulator::rakeDownMotionDone(){
	return rake_direction == RAKE_LOWERING && rake_timer->HasPeriodPassed(RAKE_TIMEOUT);
}

void Manipulator::pullTote()
{
	log->write(Log::INFO_LEVEL, "Pulling tote\n");
	left_wheel->Set(0.5);			//0.5 is an arbitrary number, may change
	right_wheel->Set(0.5);			//also, +/- is written here to signify inwards/outwards, not right/left (check, may need to change)
}

void Manipulator::pushTote()
{
	log->write(Log::INFO_LEVEL, "Pushing tote\n");
	left_wheel->Set(-0.2);
	right_wheel->Set(-0.2);
}

void Manipulator::closeFlaps(bool close)
{
	//close or open based on value of close
	if (close && (!flaps_opened_limit->Get() && using_limits)) {	//close flaps
		log->write(Log::INFO_LEVEL, "Closing flaps\n");
		close_flaps->Set(0.5);
		flap_state = FLAP_CLOSING;
	}
	else if (!close && (!flaps_closed_limit->Get() && using_limits)) {		//open flaps
		log->write(Log::INFO_LEVEL, "Opening flaps\n");
		close_flaps->Set(-0.5);
		flap_state = FLAP_OPENING;
	}
	flap_timer->Start();
	flap_timer->Reset();
}

void Manipulator::setSurface(float s)
{
	if (surface != s) {
		log->write(Log::INFO_LEVEL,"Changed surface height to %f", s);
	}
	surface = s;
}

void Manipulator::setTargetLevel(int level)
{
	log->write(Log::INFO_LEVEL, "Set lifter preset to %d", level);
	int new_target = level * TOTE_HEIGHT + surface;	//surface = height of surface on which we are trying to stack totes ((private variable))
	if (abs(current_height - new_target) < abs(current_height - target_height)) {		//in case of button mash, go to whichever instruction is closest to current position
		target_height = new_target;
	}
	lifter_timeout = (float)(target_height - current_height) * LEVEL_TIMEOUT;
	lift_timer->Start();
	lift_timer->Reset();
}

float Manipulator::getHeight()
{
	return current_height;
}

int Manipulator::getLevel()
{
	return (current_height - surface) / TOTE_HEIGHT;
}

/*
 * removing this since it messes with timeout logic
void Manipulator::changeHeight(float change)
{
	//this overrides whatever the previous target height was
	target_height = current_height + change;
}
*/

void Manipulator::spinTote(float direction)
{
	//might swap left and right depending on which twist direction the joysticks consider positive
	float left_dir = 0.5-direction;		//totally random (unrelated to 0.5 in pullTote)
	float right_dir = 0.5+direction;

	//neither motor can go above 1.0, and for now neither goes below neutral
	if (left_dir<0.0) {
		left_dir = 0.0;
	}
	else if (left_dir>1.0) {
		left_dir = 1.0;
	}
	if (right_dir<0.0) {
		right_dir = 0.0;
	}
	else if (right_dir>1.0) {
		right_dir=1.0;
	}
	log->write(Log::INFO_LEVEL, "Spinning tote: Direction = %f\tLeft = %f\tRight = %f\n", direction, left_dir, right_dir);
	left_wheel->Set(left_dir);
	right_wheel->Set(right_dir);
}

/*void Manipulator::startConveyorBelt() {
	belt_moving = true;
}

void Manipulator::stopConveyorBelt() {
	belt_moving = false;
}*/

void Manipulator::honorLimits(bool to_use_or_not_to_use)
{
	if(to_use_or_not_to_use){
		log->write(Log::INFO_LEVEL, "Using limits\n");
	}
	else{
		log->write(Log::INFO_LEVEL, "Not using limits\n");
	}
	using_limits = to_use_or_not_to_use;
}

void Manipulator::liftLifters(lifter_direction direction)
{
	if (direction == MOVING_UP && (!lift_upper_limit->Get() || !using_limits) ) {
		log->write(Log::INFO_LEVEL, "Lift moving up\n");
		lifter_one->Set(0.5);
		lifter_two->Set(0.5);
	}

	else if (direction == MOVING_DOWN && (!lift_lower_limit->Get() || !using_limits) ) {
		log->write(Log::INFO_LEVEL, "Lift moving down\n");
		lifter_one->Set(-0.5);
		lifter_two->Set(-0.5);

	}
	else if (direction == NOT_MOVING) {
		log->write(Log::INFO_LEVEL, "Lift motors stopped\n");
		lifter_one->Set(0.0);
		lifter_two->Set(0.0);
	}
}

void Manipulator::liftRakes(bool going_up)
{
	if (going_up) {
		if (!port_rake_limit->Get() && using_limits) {
			log->write(Log::INFO_LEVEL, "Port Rake moving up\n");
			rake_port ->Set(0.5);				//find out from end effector whether limits are on top or on bottom
		}
		if (!starboard_rake_limit->Get() && using_limits) {
			log->write(Log::INFO_LEVEL, "Starboard Rake moving up\n");
			rake_starboard ->Set(0.5);
		}
		rake_direction = RAKE_LIFTING;
	}
	else {
		log->write(Log::INFO_LEVEL, "Rakes moving down\n");
		rake_port->Set(-0.5);
		rake_starboard->Set(-0.5);
		rake_direction = RAKE_LOWERING;
	}
	//controls moving rakes up/down
	rake_timer->Start();
	rake_timer->Reset();
}

bool Manipulator::isInsignificantChange(float first, float second)
{
	return fabs(first - second) < LIFTER_RANGE;
}
