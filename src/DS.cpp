#include <WPILib.h>
#include <Encoder.h>
#include "Ports.h"
#include "DS.h"
#include "Log.h"
#include <CameraServer.h>
DS* DS::INSTANCE = NULL;

DS::DS() {
	encoder = new Encoder(RobotPorts::ENCODER_A, RobotPorts::ENCODER_B);

	od_xaxis_encoder = new Encoder(RobotPorts::OD_XAXIS_ENCODER_A,
			RobotPorts::OD_XAXIS_ENCODER_B);
	od_yaxis_encoder = new Encoder(RobotPorts::OD_YAXIS_ENCODER_A,
			RobotPorts::OD_YAXIS_ENCODER_B);

	joystick_one = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	joystick_two = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	joystick_three = Joystick::GetStickForPort(DSPorts::BUTTONS_JOYSTICK);

	front_left_motor = new PortsByMotor(JoystickPorts::GO_FORWARDS_FRONT_LEFT_MOTOR, JoystickPorts::GO_BACKWARDS_FRONT_LEFT_MOTOR, joystick_one);
	front_right_motor = new PortsByMotor(JoystickPorts::GO_FORWARDS_FRONT_RIGHT_MOTOR, JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR, joystick_one);
	rear_left_motor = new PortsByMotor(JoystickPorts::GO_FORWARDS_REAR_LEFT_MOTOR, JoystickPorts::GO_BACKWARDS_REAR_LEFT_MOTOR, joystick_one);
	rear_right_motor = new PortsByMotor(JoystickPorts::GO_FORWARDS_REAR_RIGHT_MOTOR, JoystickPorts::GO_BACKWARDS_REAR_RIGHT_MOTOR, joystick_one);

	lift_motor = new PortsByMotor(JoystickPorts::GOING_UP_LIFTER, JoystickPorts::GOING_DOWN_LIFTER, joystick_two);
	flap_motor = new PortsByMotor(JoystickPorts::CLOSE_FLAPS, JoystickPorts::OPEN_FLAPS, joystick_one);
	port_rake_motor = new PortsByMotor(JoystickPorts::LIFT_PORT_RAKE, JoystickPorts::LOWER_PORT_RAKE, joystick_two);
	starboard_rake_motor = new PortsByMotor(JoystickPorts::LIFT_STARBOARD_RAKE, JoystickPorts::LOWER_STARBOARD_RAKE, joystick_two);
	left_tote_motor = new PortsByMotor(JoystickPorts::PULL_IN_LEFT_WHEEL, JoystickPorts::PUSH_OUT_LEFT_WHEEL, joystick_two);
	right_tote_motor = new PortsByMotor(JoystickPorts::PULL_IN_RIGHT_WHEEL, JoystickPorts::PUSH_OUT_RIGHT_WHEEL, joystick_two);

	log = Log::getInstance();
}

void DS::process() {
	/*
	 log->write(Log::TRACE_LEVEL, "-----------Talon Encoder-------------\n");
	 log->write(Log::INFO_LEVEL, "Distance: %i\n", rear_left_wheel->GetEncPosition());
	 log->write(Log::INFO_LEVEL, "Velocity: %i\n", rear_left_wheel->GetEncVel());

	 log->write(Log::TRACE_LEVEL, "----------Digital Encoder------------\n");
	 */
	log->write(Log::INFO_LEVEL, "Get: %i\n", encoder->Get());
	log->write(Log::INFO_LEVEL, "Get Raw: %i\n", encoder->GetRaw());
	log->write(Log::INFO_LEVEL, "Rate: %f\n", encoder->GetRate());
	//log->write(Log::INFO_LEVEL, "Get Encoding Scale: %i\n", encoder->GetEncodingScale());

	log->write(Log::INFO_LEVEL, "Get: Front Left Encoder %i\n",
			od_xaxis_encoder->Get());
	log->write(Log::INFO_LEVEL, "Get: Back Right Encoder %i\n",
			od_yaxis_encoder->Get());


}


//got moved to PortsByMotor
/*DS::Direction DS::getDirection(PortsByMotor* motor) {
	if (motor->joystick->GetRawButton(motor.PORT_1)) {
		if (!motor.joystick->GetRawButton(motor.PORT_2)) {
			return FORWARD;
		}
	}
	else if (motor.joystick->GetRawButton(motor.PORT_2)) {
		return BACKWARD;
	}
	return STILL;
}*/

DS* DS::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new DS();
	}
	return INSTANCE;
}
