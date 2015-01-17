#include <WPILib.h>
#include <CANTalon.h>
#include "Manipulator.h"
#include "Ports.h"
#include <cmath>

Manipulator* Manipulator::INSTANCE = NULL;

Manipulator::Manipulator()
{
	mobility = Mobility::getInstance();

	left_wheel = new CANTalon(RobotPorts::LEFT_WHEEL);
	right_wheel = new CANTalon(RobotPorts::RIGHT_WHEEL);
	lifter_one = new CANTalon(RobotPorts::LIFTER_ONE);
	lifter_two = new CANTalon(RobotPorts::LIFTER_TWO);
	rake_port = new CANTalon(RobotPorts::RAKE_PORT_MOTOR);
	rake_starboard = new CANTalon(RobotPorts::RAKE_STARBOARD_MOTOR);
	close_hooks = new CANTalon(RobotPorts::CLOSE_HOOKS_MOTOR);
	current_height = 0; //starting height (floor level)
	target_height = 0;
	using_limits = true;
	belt_moving = false;

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



}

void Manipulator::pullTote()
{
	left_wheel->Set(0.5);
	right_wheel->Set(0.5);
}

void Manipulator::pushTote(){
	left_wheel->Set(-0.2);
	right_wheel->Set(-0.2);
}
void Manipulator::setHooks(bool close) {
	//close or open based on value of close
}
void Manipulator::setTargetHeight(int level,bool on_step) {
	int new_target = level*TOTE_HEIGHT;
	if(on_step){
		new_target+=2.0;
	}
	if(abs(current_height - new_target) < abs(current_height - target_height)){
		target_height = new_target;
	}
}
float Manipulator::getHeight(){
	return current_height;
}
int Manipulator::getLevel(){
	return current_height/TOTE_HEIGHT;
}
void Manipulator::changeHeight(float change){
	//this overrides whatever the previous target height was
	target_height = current_height + change;
}
void Manipulator::spinTote(float direction){
	//might swap left and right depending on which twist direction the joysticks consider positive
	float left_dir = 0.5-direction;
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
void Manipulator::startConveyorBelt() {
	belt_moving = true;
}
void Manipulator::stopConveyorBelt() {
	belt_moving = false;
}
void Manipulator::honor_limits(bool to_use_or_not_to_use) {
	using_limits = to_use_or_not_to_use;
}
void Manipulator::read_limits() {
	//some_limit = arduino->get_some_limit();
}
void Manipulator::liftLifters()
{
	lifter_one ->Set(0.5);
	lifter_two->Set(0.5);
}
void Manipulator::liftRakes()
{
	rake_port ->Set(0.5);
	rake_starboard ->Set(0.5);
	//controls moving rakes up/down
}

