#include <WPILib.h>
#include <CANTalon.h>
#include "Manipulator.h"
#include "Ports.h"
#include <cmath>

Manipulator* Manipulator::INSTANCE = NULL;
const float Manipulator::TOTE_HEIGHT = 12.1;
const float Manipulator::FLOOR = 0.0;
const float Manipulator::SCORING_PLATFORM = 2.0;
const float Manipulator::STEP = 6.25;
const int Manipulator::pulse_per_rev = 64;
const float Manipulator::inch_per_rev = 4;

Manipulator::Manipulator()
{
	mobility = Mobility::getInstance();

	left_wheel = new CANTalon(RobotPorts::LEFT_WHEEL);
	right_wheel = new CANTalon(RobotPorts::RIGHT_WHEEL);
	lifter_one = new CANTalon(RobotPorts::LIFTER_ONE);
	lifter_two = new CANTalon(RobotPorts::LIFTER_TWO);
	rake_port = new CANTalon(RobotPorts::RAKE_PORT_MOTOR);
	rake_starboard = new CANTalon(RobotPorts::RAKE_STARBOARD_MOTOR);
	close_flaps = new CANTalon(RobotPorts::CLOSE_FLAPS_MOTOR);
	lift_upper_limit = new DigitalInput(RobotPorts::LIFT_UPPER_LIMIT);
	lift_lower_limit = new DigitalInput(RobotPorts::LIFT_LOWER_LIMIT);
	flaps_upper_limit = new DigitalInput(RobotPorts::FLAPS_UPPER_LIMIT);
	flaps_lower_limit = new DigitalInput(RobotPorts::FLAPS_LOWER_LIMIT);
	port_rake_limit = new DigitalInput(RobotPorts::PORT_RAKE_LIMIT);
	starboard_rake_limit = new DigitalInput(RobotPorts::STARBOARD_RAKE_LIMIT);
	encoder = new Encoder(RobotPorts::ENCODER_A, RobotPorts::ENCODER_B);
	encoder->SetDistancePerPulse(inch_per_rev/pulse_per_rev);	//inches per revolution / pulses per revolution = inches per pulse
	current_height = 0; //starting height (floor level)
	target_height = 0;
	using_limits = true;
	//belt_moving = false;
	surface = 0;
}

Manipulator::~Manipulator() {
	// TODO Auto-generated destructor stub
}


Manipulator* Manipulator::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new Manipulator();
	}
	return INSTANCE;
}


void Manipulator::process() {
	current_height = encoder->GetDistance();	//uses data from encoder to determine current height of lift

	if (current_height < target_height + .3 && current_height > target_height - .3)	//insignificant change
	{
		lifter_one->Set(0);
		lifter_two->Set(0);
	}
	else		//significant change
	{
		if(current_height < target_height && (!lift_upper_limit && using_limits)){		//desired height is above current height and upper limit has not been reached
			lifter_one ->Set(0.5);
			lifter_two ->Set(0.5);
		}

		else if(current_height > target_height && (!lift_lower_limit && using_limits)){		//desired height is below current height
			lifter_one -> Set(-0.5);
			lifter_two -> Set(-0.5);
		}
	}

	if(lift_lower_limit){		//reset encoder to 0 every time lift hits lower limit switch
		encoder->Reset();
	}
}


void Manipulator::pullTote()
{
	left_wheel->Set(0.5);			//0.5 is an arbitrary number, may change
	right_wheel->Set(0.5);			//also, +/- is written here to signify inwards/outwards, not right/left (check, may need to change)
}

void Manipulator::pushTote(){
	left_wheel->Set(-0.2);
	right_wheel->Set(-0.2);
}

void Manipulator::setFlaps(bool close) {
	//close or open based on value of close
	if(close && (!flaps_lower_limit && using_limits)){	//close flaps
		close_flaps->Set(0.5);
	}
	else if (!close && (!flaps_upper_limit && using_limits)){		//open flaps
		close_flaps->Set(-0.5);
	}
}

void Manipulator::setSurface(float s){
	surface = s;
}

/*Please destroy this
int Manipulator::getSurface() {
	if (surface == FLOOR){
		return 0;
	}
	else if(surface == SCORING_PLATFORM) {
		return 1;
	}
	else {				//if(surface == STEP)
		return 2;
	}
}
*/

void Manipulator::setTargetHeight(int level) {
	int new_target = level*TOTE_HEIGHT + surface;	//surface = height of surface on which we are trying to stack totes ((private variable))
	if(abs(current_height - new_target) < abs(current_height - target_height)){		//in case of button mash, go to whichever instruction is closest to current position
		target_height = new_target;
	}
}

float Manipulator::getHeight(){
	return current_height;
}

int Manipulator::getLevel(){
	return (current_height - surface)/TOTE_HEIGHT;
}

void Manipulator::changeHeight(float change){
	//this overrides whatever the previous target height was
	target_height = current_height + change;
}

void Manipulator::spinTote(float direction){
	//might swap left and right depending on which twist direction the joysticks consider positive
	float left_dir = 0.5-direction;		//totally random (unrelated to 0.5 in pullTote)
	float right_dir = 0.5+direction;

	//neither motor can go above 1.0, and for now neither goes below neutral
	if(left_dir<0.0){
		left_dir = 0.0;
	}
	else if(left_dir>1.0){
		left_dir = 1.0;
	}
	if(right_dir<0.0){
		right_dir = 0.0;
	}
	else if(right_dir>1.0){
		right_dir=1.0;
	}
	left_wheel->Set(left_dir);
	right_wheel->Set(right_dir);
}

/*void Manipulator::startConveyorBelt() {
	belt_moving = true;
}

void Manipulator::stopConveyorBelt() {
	belt_moving = false;
}*/

void Manipulator::honor_limits(bool to_use_or_not_to_use) {
	using_limits = to_use_or_not_to_use;
}

void Manipulator::liftLifters()
{
	if(!lift_upper_limit && using_limits) {
		lifter_one ->Set(0.5);
		lifter_two->Set(0.5);
	}
}

void Manipulator::liftRakes()
{
	if(!port_rake_limit && using_limits){
		rake_port ->Set(0.5);
	}
	if(!starboard_rake_limit && using_limits){
		rake_starboard ->Set(0.5);
	}
	//controls moving rakes up/down
}

