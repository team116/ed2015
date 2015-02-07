#include <WPILib.h>
#include <CANTalon.h>
#include "Manipulator.h"
#include "Ports.h"
#include <cmath>
#include <ctime>

Manipulator* Manipulator::INSTANCE = NULL;

// TODO: get actual timeouts
const float Manipulator::FLAP_TIMEOUT = 1.1;
const float Manipulator::RAKE_TIMEOUT = 1.1;
const float Manipulator::LEVEL_TIMEOUT = 1.1; // the amount of time to be given for each level of movement
const float Manipulator::WHEEL_TIMEOUT = 1.1;

// lifter stuff
const float Manipulator::LIFTER_RANGE = 0.3; // the acceptable height range for our presets
const int Manipulator::PULSE_PER_REV = 64;
const float Manipulator::INCH_PER_REV = 4;

const float Manipulator::TOTE_HEIGHT = 12.1;
const float Manipulator::FLOOR = 0.0;
const float Manipulator::SCORING_PLATFORM = 2.0;
const float Manipulator::STEP = 6.25;
const float Manipulator::FLAP_ANGLE_HIGH = 270;
const float Manipulator::FLAP_ANGLE_MID = 135; 	//note: this will change probably
const float Manipulator::FLAP_ANGLE_LOW = 0;
const float Manipulator::FLAP_RANGE = 10; 	//note: this will change probably

Manipulator::Manipulator()
{
	// subsystem instance getting
	mobility = Mobility::getInstance();
	log = Log::getInstance();

	left_wheel = new CANTalon(RobotPorts::LEFT_WHEEL);
	right_wheel = new CANTalon(RobotPorts::RIGHT_WHEEL);
	wheel_timer = new Timer();
	wheel_state = WHEELS_STILL;

	// lifter initializations
	lifter_one = new CANTalon(RobotPorts::LIFTER_ONE);
	lifter_two = new CANTalon(RobotPorts::LIFTER_TWO);
	lift_upper_limit = new DigitalInput(RobotPorts::LIFT_UPPER_LIMIT);
	lift_lower_limit = new DigitalInput(RobotPorts::LIFT_LOWER_LIMIT);
	encoder = new Encoder(RobotPorts::ENCODER_A, RobotPorts::ENCODER_B);
	encoder->SetDistancePerPulse(INCH_PER_REV/PULSE_PER_REV);	//inches per revolution / pulses per revolution = inches per pulse
	potentiometer= new AnalogPotentiometer(RobotPorts::FLAP_POTENTIOMETER, 270, 0); //270 = full range of positions; 0 = lowest position
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
	flap_pos = FLAP_LOW;	//note: this might change idk

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
	log->write(Log::TRACE_LEVEL, "%s\tCurrent Height: %f \nTarget Height: %f\n", Utils::getCurrentTime(), current_height,target_height);

	if(pushToteDone()) {
		log->write(Log::TRACE_LEVEL, "%s\ttote pushed\n", Utils::getCurrentTime());
		left_wheel->Set(0.0);
		right_wheel->Set(0.0);
		wheel_state = WHEELS_STILL;
	}

	if(pullToteDone()) {
		log->write(Log::TRACE_LEVEL, "%s\tTote pulled\n", Utils::getCurrentTime());
		left_wheel->Set(0.0);
		right_wheel->Set(0.0);
		wheel_state = WHEELS_STILL;
	}

	if (flapMotionDone()) {
		if(flap_state == FLAP_CLOSING){
			log->write(Log::TRACE_LEVEL, "%s\tFlaps closed\n", Utils::getCurrentTime());
		}
		else{
			log->write(Log::TRACE_LEVEL, "%s\tFlaps opened\n", Utils::getCurrentTime());
		}
		close_flaps->Set(0.0);
		flap_state = FLAP_STILL;
	}
	else {
		switch(flap_pos){
		case FLAP_LOW:
			closeFlaps(true);
			break;
		case FLAP_MID:
			if (potentiometer->Get() < FLAP_ANGLE_MID) {	//TODO: check to make sure orientation is correct (aka small value from potentiometer = more closed)
				closeFlaps(false);
			}
			else {
				closeFlaps(true);
			}
			break;
		case FLAP_HIGH:
			closeFlaps(false);
			break;
		}
	}


	if (isInsignificantChange(current_height, target_height)) {
		log->write(Log::TRACE_LEVEL, "%s\tChange insignificant: lift motors stopped\n", Utils::getCurrentTime());
		lifter_one->Set(0);
		lifter_two->Set(0);
	}
	else {
		if(canMoveLifter()) {
			if(current_height < target_height) {
				log->write(Log::TRACE_LEVEL, "%s\tMoving lift up\n", Utils::getCurrentTime());
				lifter_one->Set(0.5);
				lifter_two->Set(0.5);
			}
			else {
				log->write(Log::TRACE_LEVEL, "%s\tMoving lift down\n", Utils::getCurrentTime());
				lifter_one->Set(-0.5);
				lifter_two->Set(-0.5);
			}
		}
	}


	if (lift_lower_limit->Get()){		//reset encoder to 0 every time lift hits lower limit switch
		log->write(Log::TRACE_LEVEL, "%s\tHit bottom of lift: encoder set to 0\n", Utils::getCurrentTime());
		encoder->Reset();
	}

	if (rakeUpMotionDone()){ 	//TODO: get real times
		log->write(Log::TRACE_LEVEL, "%s\tRake finished moving up\n", Utils::getCurrentTime());
		rake_port->Set(0.0);
		rake_starboard->Set(0.0);
		rake_direction = RAKE_STILL;
	}
	else if (rakeDownMotionDone()){ 	//TODO: get real timeout period
		log->write(Log::TRACE_LEVEL, "%s\tRake finished moving down\n", Utils::getCurrentTime());
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

bool Manipulator::flapMotionDone(){	//TODO: add timeouts to flap positions
	float posi = potentiometer->Get();
	switch(flap_pos){
	case FLAP_LOW:
		return (fabs(posi - FLAP_ANGLE_LOW) < FLAP_RANGE);
	case FLAP_MID:
		return (fabs(posi - FLAP_ANGLE_MID) < FLAP_RANGE);
	case FLAP_HIGH:
		return (fabs(posi - FLAP_ANGLE_HIGH) < FLAP_RANGE);
	}
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

bool Manipulator::pushToteDone() {
	return wheel_state == WHEELS_PUSHING && wheel_timer->HasPeriodPassed(WHEEL_TIMEOUT);
}

bool Manipulator::pullToteDone() {
	return wheel_state == WHEELS_PULLING && wheel_timer->HasPeriodPassed(WHEEL_TIMEOUT);
}

void Manipulator::pullTote()
{
	log->write(Log::TRACE_LEVEL, "%s\tPulling tote\n", Utils::getCurrentTime());
	wheel_state = WHEELS_PULLING;
	left_wheel->Set(0.5);			//0.5 is an arbitrary number, may change
	right_wheel->Set(0.5);			//also, +/- is written here to signify inwards/outwards, not right/left (check, may need to change)
	wheel_timer->Start();
	wheel_timer->Reset();
}

void Manipulator::pushTote()
{
	log->write(Log::TRACE_LEVEL, "%s\tPushing tote\n", Utils::getCurrentTime());
	wheel_state = WHEELS_PUSHING;
	left_wheel->Set(-0.2);
	right_wheel->Set(-0.2);
	wheel_timer->Start();
	wheel_timer->Reset();
}

void Manipulator::closeFlaps(bool close)
{
	//close or open based on value of close
	if (close && (!flaps_opened_limit->Get() && using_limits)) {	//close flaps
		log->write(Log::TRACE_LEVEL, "%s\tClosing flaps\n", Utils::getCurrentTime());
		close_flaps->Set(0.5);
		flap_state = FLAP_CLOSING;
	}
	else if (!close && (!flaps_closed_limit->Get() && using_limits)) {		//open flaps
		log->write(Log::TRACE_LEVEL, "%s\tOpening flaps\n", Utils::getCurrentTime());
		close_flaps->Set(-0.5);
		flap_state = FLAP_OPENING;
	}
	flap_timer->Start();
	flap_timer->Reset();
}

void Manipulator::setSurface(float s)
{
	if (surface != s) {
		log->write(Log::TRACE_LEVEL,"%s\tChanged surface height to %f", Utils::getCurrentTime(), s);
	}
	surface = s;
}

void Manipulator::setTargetLevel(int level)
{
	log->write(Log::TRACE_LEVEL, "%s\tSet lifter preset to %d", Utils::getCurrentTime(), level);
	int new_target = level * TOTE_HEIGHT + surface;	//surface = height of surface on which we are trying to stack totes ((private variable))
	if (abs(current_height - new_target) < abs(current_height - target_height)) {		//in case of button mash, go to whichever instruction is closest to current position
		target_height = new_target;
	}
	lifter_timeout = (float)(target_height - current_height) * LEVEL_TIMEOUT;
	lift_timer->Start();
	lift_timer->Reset();
}

void Manipulator::setFlapPosition(flap_position p){
	log->write(Log::TRACE_LEVEL, "flaps set to position %i\n", p);
	flap_pos = p;
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
	log->write(Log::TRACE_LEVEL, "%s\tSpinning tote: Direction = %f\n", Utils::getCurrentTime(), direction, left_dir, right_dir);
	left_wheel->Set(left_dir);
	right_wheel->Set(right_dir);
}

void Manipulator::honorLimits(bool to_use_or_not_to_use)
{
	if(to_use_or_not_to_use){
		log->write(Log::INFO_LEVEL, "%s\tStarted using limits\n", Utils::getCurrentTime());
	}
	else{
		log->write(Log::INFO_LEVEL, "%s\tStopped using limits\n", Utils::getCurrentTime());
	}
	using_limits = to_use_or_not_to_use;
}

void Manipulator::liftLifters(lifter_direction direction)
{
	if (direction == MOVING_UP && (!lift_upper_limit->Get() || !using_limits) ) {
		log->write(Log::TRACE_LEVEL, "%s\tLift moving up\n", Utils::getCurrentTime());
		lifter_one->Set(0.5);
		lifter_two->Set(0.5);
	}

	else if (direction == MOVING_DOWN && (!lift_lower_limit->Get() || !using_limits) ) {
		log->write(Log::TRACE_LEVEL, "%s\tLift moving down\n", Utils::getCurrentTime());
		lifter_one->Set(-0.5);
		lifter_two->Set(-0.5);

	}
	else if (direction == NOT_MOVING) {
		log->write(Log::TRACE_LEVEL, "%s\tLift motors stopped\n", Utils::getCurrentTime());
		lifter_one->Set(0.0);
		lifter_two->Set(0.0);
	}
}

void Manipulator::liftRakes(bool going_up)
{
	if (going_up) {
		if (!port_rake_limit->Get() && using_limits) {
			log->write(Log::TRACE_LEVEL, "%s\tPort Rake moving up\n", Utils::getCurrentTime());
			rake_port ->Set(0.5);				//find out from end effector whether limits are on top or on bottom
		}
		if (!starboard_rake_limit->Get() && using_limits) {
			log->write(Log::TRACE_LEVEL, "%s\tStarboard Rake moving up\n", Utils::getCurrentTime());
			rake_starboard ->Set(0.5);
		}
		rake_direction = RAKE_LIFTING;
	}
	else {
		log->write(Log::TRACE_LEVEL, "%s\tRakes moving down\n", Utils::getCurrentTime());
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
