#include <WPILib.h>
#include <Encoder.h>
#include <Timer.h>
#include "Ports.h"
#include "DS.h"
#include "Log.h"
#include <CameraServer.h>
DS* DS::INSTANCE = NULL;

DS::DS()
{
	log = Log::getInstance();

	encoder = new Encoder(RobotPorts::ENCODER_A, RobotPorts::ENCODER_B);

	od_xaxis_encoder = new Encoder(RobotPorts::OD_XAXIS_ENCODER_A, RobotPorts::OD_XAXIS_ENCODER_B);
	od_yaxis_encoder = new Encoder(RobotPorts::OD_YAXIS_ENCODER_A, RobotPorts::OD_YAXIS_ENCODER_B);

	joystick_one = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	joystick_two = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	joystick_three = Joystick::GetStickForPort(DSPorts::BUTTONS_JOYSTICK);

	log_timer = new Timer();
	log_timer->Start();
}

void DS::process() {
	/*
	 log->write(Log::TRACE_LEVEL, "-----------Talon Encoder-------------\n");
	 log->write(Log::INFO_LEVEL, "Distance: %i\n", rear_left_wheel->GetEncPosition());
	 log->write(Log::INFO_LEVEL, "Velocity: %i\n", rear_left_wheel->GetEncVel());

	 log->write(Log::TRACE_LEVEL, "----------Digital Encoder------------\n");
	 */
	if (log_timer->HasPeriodPassed(5.0)) {
		log->write(Log::INFO_LEVEL, "Get: %i\n", encoder->Get());
		log->write(Log::INFO_LEVEL, "Get Raw: %i\n", encoder->GetRaw());
		log->write(Log::INFO_LEVEL, "Rate: %f\n", encoder->GetRate());
	}
	// log->write(Log::INFO_LEVEL, "Get Encoding Scale: %i\n", encoder->GetEncodingScale());

	// log->write(Log::INFO_LEVEL, "Get: Front Left Encoder %i\n", od_xaxis_encoder->Get());
	// log->write(Log::INFO_LEVEL, "Get: Back Right Encoder %i\n", od_yaxis_encoder->Get());

}

DS::Direction DS::getDirection(Joystick* joystick, int port_1, int port_2) {
	if (joystick->GetRawButton(port_1)) {
		if (!joystick->GetRawButton(port_2)) {
			return FORWARD;
		}
	}
	else if (joystick->GetRawButton(port_2)) {
		return BACKWARD;
	}
	return STILL;
}

DS::Direction DS::frontLeftMotorDirection() {
	return getDirection(joystick_one, JoystickPorts::GO_FORWARDS_FRONT_LEFT_MOTOR, JoystickPorts::GO_BACKWARDS_FRONT_LEFT_MOTOR);
}
DS::Direction DS::frontRightMotorDirection() {
	return getDirection(joystick_one, JoystickPorts::GO_FORWARDS_FRONT_RIGHT_MOTOR, JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR);
}
DS::Direction DS::rearLeftMotorDirection() {
	return getDirection(joystick_one, JoystickPorts::GO_FORWARDS_REAR_LEFT_MOTOR, JoystickPorts::GO_BACKWARDS_REAR_LEFT_MOTOR);
}
DS::Direction DS::rearRightMotorDirection() {
	return getDirection(joystick_one, JoystickPorts::GO_FORWARDS_REAR_RIGHT_MOTOR, JoystickPorts::GO_BACKWARDS_REAR_RIGHT_MOTOR);
}
DS::Direction DS::lifterMotorDirection() {
	return getDirection(joystick_two, JoystickPorts::GOING_UP_LIFTER, JoystickPorts::GOING_DOWN_LIFTER);
}
DS::Direction DS::armFlapMotorDirection() {
	return getDirection(joystick_one, JoystickPorts::CLOSE_FLAPS, JoystickPorts::OPEN_FLAPS);
}	//note: forward = closing, backward = opening
DS::Direction DS::portRakeMotorDirection() {
	return getDirection(joystick_two, JoystickPorts::LIFT_PORT_RAKE, JoystickPorts::LOWER_PORT_RAKE);
}
DS::Direction DS::starboardRakeMotorDirection() {
	return getDirection(joystick_two, JoystickPorts::LIFT_STARBOARD_RAKE, JoystickPorts::LOWER_STARBOARD_RAKE);
}
DS::Direction DS::leftToteWheelDirection() {
	return getDirection(joystick_two, JoystickPorts::PULL_IN_LEFT_WHEEL, JoystickPorts::PUSH_OUT_LEFT_WHEEL);
}
DS::Direction DS::rightToteWheelDirection() {
	return getDirection(joystick_two, JoystickPorts::PULL_IN_RIGHT_WHEEL, JoystickPorts::PUSH_OUT_RIGHT_WHEEL);
}
DS::Direction DS::leftTrexDirection() {
    return getDirection(joystick_three, JoystickPorts::LEFT_TREX_IN, JoystickPorts::LEFT_TREX_OUT);
}
DS::Direction DS::rightTrexDirection() {
    return getDirection(joystick_three, JoystickPorts::RIGHT_TREX_IN, JoystickPorts::RIGHT_TREX_OUT);
}
DS::Direction DS::leftRakeArmDirection() {
    return getDirection(joystick_three, JoystickPorts::LEFT_RAKE_ARM_IN, JoystickPorts::LEFT_RAKE_ARM_OUT);
}
DS::Direction DS::rightRakeArmDirection() {
    return getDirection(joystick_three, JoystickPorts::RIGHT_RAKE_ARM_IN, JoystickPorts::RIGHT_RAKE_ARM_OUT);
}


DS* DS::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new DS();
	}
	return INSTANCE;
}
