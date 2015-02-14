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

	front_left_wheel = new CANTalon(RobotPorts::FRONT_LEFT_MOTOR);
	front_right_wheel = new CANTalon(RobotPorts::FRONT_RIGHT_MOTOR);
	rear_left_wheel = new CANTalon(RobotPorts::REAR_LEFT_MOTOR);
	rear_right_wheel = new CANTalon(RobotPorts::REAR_RIGHT_MOTOR);
	left_grabber_wheel = new CANTalon(RobotPorts::LEFT_WHEEL);
	right_grabber_wheel = new CANTalon(RobotPorts::RIGHT_WHEEL);

	lifter_one = new CANTalon(RobotPorts::LIFTER_ONE);
	lifter_two = new CANTalon(RobotPorts::LIFTER_TWO);

	rake_port = new CANTalon(RobotPorts::RAKE_PORT_MOTOR);
	rake_starboard = new CANTalon(RobotPorts::RAKE_STARBOARD_MOTOR);

	flaps = new CANTalon(RobotPorts::CLOSE_FLAPS_MOTOR);

	joystick_one = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	joystick_two = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	joystick_three = Joystick::GetStickForPort(DSPorts::BUTTONS_JOYSTICK);

	log = Log::getInstance();
}

void DS::process() {
	// a !XOR would be nice here... would get rid of the else at the end, but I think this is more clear
	// could do !(button1 ^ button2)

	/*//following has been moved to getArmFlapDirection() for the logic and Motors.cpp for actually moving
	 if (joystick_one->GetRawButton(JoystickPorts::CLOSE_FLAPS)
	 && joystick_one->GetRawButton(JoystickPorts::OPEN_FLAPS)) {
	 flaps->Set(0.0);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::CLOSE_FLAPS)) {
	 flaps->Set(0.5);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::OPEN_FLAPS)) {
	 flaps->Set(-0.5);
	 }
	 else {
	 flaps->Set(0.0);
	 }
	 */

	//wheels
	/*
	 if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_LEFT_MOTOR) &&
	 joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_FRONT_LEFT_MOTOR)) {
	 front_left_wheel->Set(0.0);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_LEFT_MOTOR)) {
	 front_left_wheel->Set(0.5);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_FRONT_LEFT_MOTOR)) {
	 front_left_wheel->Set(-0.5);
	 }
	 else {
	 front_left_wheel->Set(0.0);
	 }

	 if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_RIGHT_MOTOR) &&
	 joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR)) {
	 front_right_wheel->Set(0.0);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_RIGHT_MOTOR)) {
	 front_right_wheel->Set(0.5);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR)) {
	 front_right_wheel->Set(-0.5);
	 }
	 else {
	 front_right_wheel->Set(0.0);
	 }

	 if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_LEFT_MOTOR) &&
	 joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_REAR_LEFT_MOTOR)) {
	 rear_left_wheel->Set(0.0);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_LEFT_MOTOR)) {
	 rear_left_wheel->Set(0.5);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_REAR_LEFT_MOTOR)) {
	 rear_left_wheel->Set(-0.5);
	 }
	 else {
	 rear_left_wheel->Set(0.0);
	 }

	 if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_RIGHT_MOTOR) &&
	 joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_REAR_RIGHT_MOTOR)) {
	 rear_right_wheel->Set(0.0);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_RIGHT_MOTOR)) {
	 rear_right_wheel->Set(0.5);
	 }
	 else if (joystick_one->GetRawButton(JoystickPorts::GO_BACKWARDS_REAR_RIGHT_MOTOR)) {
	 rear_right_wheel->Set(-0.5);
	 }
	 else {
	 rear_right_wheel->Set(0.0);
	 }
	 */

	//joystick two
	//rakes
	/*if (joystick_two->GetRawButton(JoystickPorts::LIFT_STARBOARD_RAKE)
	 && joystick_two->GetRawButton(
	 JoystickPorts::LOWER_STARBOARD_RAKE)) {
	 rake_starboard->Set(0.0);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::LIFT_STARBOARD_RAKE)) {
	 rake_starboard->Set(0.5);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::LOWER_STARBOARD_RAKE)) {
	 rake_starboard->Set(-0.5);
	 }
	 else {
	 rake_starboard->Set(0.0);
	 }*/

	/*if (joystick_two->GetRawButton(JoystickPorts::LIFT_PORT_RAKE)
	 && joystick_two->GetRawButton(JoystickPorts::LOWER_PORT_RAKE)) {
	 // both up and down buttons pressed, don't move
	 rake_port->Set(0.0);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::LIFT_PORT_RAKE)) {
	 rake_port->Set(0.5);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::LOWER_PORT_RAKE)) {
	 rake_port->Set(-0.5);
	 }
	 else {
	 rake_port->Set(0.0);
	 }*/

	//manipulator grabber wheels
	/*if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_LEFT_WHEEL)
	 && joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_LEFT_WHEEL)) {
	 left_grabber_wheel->Set(0.0);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_LEFT_WHEEL)) {
	 left_grabber_wheel->Set(0.5);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_LEFT_WHEEL)) {
	 left_grabber_wheel->Set(-0.5);
	 }
	 else {
	 left_grabber_wheel->Set(0.0);
	 }*/

	/*if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_RIGHT_WHEEL)
	 && joystick_two->GetRawButton(
	 JoystickPorts::PUSH_OUT_RIGHT_WHEEL)) {
	 right_grabber_wheel->Set(0.0);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_RIGHT_WHEEL)) {
	 right_grabber_wheel->Set(0.5);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_RIGHT_WHEEL)) {
	 right_grabber_wheel->Set(-0.5);
	 }
	 else {
	 right_grabber_wheel->Set(0.0);
	 }*/

	//lifters for manipulator
	/*if (joystick_two->GetRawButton(JoystickPorts::GOING_UP_LIFTER)
	 && joystick_two->GetRawButton(JoystickPorts::GOING_DOWN_LIFTER)) {
	 lifter_one->Set(0.0);
	 lifter_two->Set(0.0);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::GOING_UP_LIFTER)) {
	 lifter_one->Set(0.5);
	 lifter_two->Set(0.5);
	 }
	 else if (joystick_two->GetRawButton(JoystickPorts::GOING_DOWN_LIFTER)) {
	 lifter_one->Set(-0.5);
	 lifter_two->Set(-0.5);
	 }
	 else {
	 lifter_one->Set(0.0);
	 lifter_two->Set(0.0);
	 }*/

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

	log->write(Log::INFO_LEVEL, "Get: Front Left Wheel Velocity %i\n",
			front_left_wheel->GetEncVel());
	log->write(Log::INFO_LEVEL, "Get: Front Right Wheel Velocity %i\n",
			front_right_wheel->GetEncVel());
	log->write(Log::INFO_LEVEL, "Get: Back Left Wheel Velocity %i\n",
			rear_left_wheel->GetEncVel());
	log->write(Log::INFO_LEVEL, "Get: Back Right Wheel Velocity %i\n",
			rear_right_wheel->GetEncVel());

	log->write(Log::INFO_LEVEL, "Get: Lifter One Position %i\n",
			lifter_one->GetEncPosition());

	log->write(Log::INFO_LEVEL, "Get: Rake Port Position %i\n",
			rake_port->GetEncPosition());
	log->write(Log::INFO_LEVEL, "Get: Rake Starboard Position %i\n",
			rake_starboard->GetEncPosition());

	log->write(Log::INFO_LEVEL, "Get: Arm Flippy Floppies Position %i\n",
			flaps->GetEncPosition());

	log->write(Log::INFO_LEVEL, "Get: Upper Lifter Limit Switch %i\n",
			lifter_one->GetForwardLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Lower Lifter Limit Switch %i\n",
			lifter_one->GetReverseLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Upper Rake Port Limit Switch %i\n",
			rake_port->GetForwardLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Lower Rake Port Limit Switch %i\n",
			rake_port->GetReverseLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Upper Rake Starboard Limit Switch %i\n",
			rake_starboard->GetForwardLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Lower Rake Starboard Limit Switch %i\n",
			rake_starboard->GetReverseLimitOK());
	log->write(Log::INFO_LEVEL,
			"Get: Upper Arm Flippy Floppies isOpen Limit Switch %i\n",
			flaps->GetForwardLimitOK());
	log->write(Log::INFO_LEVEL,
			"Get: Lower Arm Flippy Floppies isClosed Limit Switch %i\n",
			flaps->GetReverseLimitOK());
	log->write(Log::INFO_LEVEL, "Get: Arm potentiometer %i\n",
			flaps->GetAnalogIn());
}

DS::Direction DS::frontRightMotorDirection() {
	if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_RIGHT_MOTOR)
			&& joystick_one->GetRawButton(
					JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR)) {
		return STILL;
	}
	else if (joystick_one->GetRawButton(
			JoystickPorts::GO_FORWARDS_FRONT_RIGHT_MOTOR)) {
		return FORWARD;
	}
	else if (joystick_one->GetRawButton(
			JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR)) {
		return BACKWARD;
	}
	else {
		return STILL;
	}
}

DS::Direction DS::frontLeftMotorDirection() {
	if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_FRONT_LEFT_MOTOR)
			&& joystick_one->GetRawButton(
					JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR)) {
		return STILL;
	}
	else if (joystick_one->GetRawButton(
			JoystickPorts::GO_FORWARDS_FRONT_LEFT_MOTOR)) {
		return FORWARD;
	}
	else if (joystick_one->GetRawButton(
			JoystickPorts::GO_BACKWARDS_FRONT_LEFT_MOTOR)) {
		return BACKWARD;
	}
	else {
		return STILL;
	}
}

DS::Direction DS::rearRightMotorDirection() {
	if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_RIGHT_MOTOR)
			&& joystick_one->GetRawButton(
					JoystickPorts::GO_BACKWARDS_REAR_RIGHT_MOTOR)) {
		return STILL;
	}
	else if (joystick_one->GetRawButton(
			JoystickPorts::GO_FORWARDS_REAR_RIGHT_MOTOR)) {
		return FORWARD;
	}
	else if (joystick_one->GetRawButton(
			JoystickPorts::GO_BACKWARDS_REAR_RIGHT_MOTOR)) {
		return BACKWARD;
	}
	else {
		return STILL;
	}
}

DS::Direction DS::rearLeftMotorDirection() {
	if (joystick_one->GetRawButton(JoystickPorts::GO_FORWARDS_REAR_LEFT_MOTOR)
			&& joystick_one->GetRawButton(
					JoystickPorts::GO_BACKWARDS_FRONT_RIGHT_MOTOR)) {
		return STILL;
	}
	else if (joystick_one->GetRawButton(
			JoystickPorts::GO_FORWARDS_REAR_LEFT_MOTOR)) {
		return FORWARD;
	}
	else if (joystick_one->GetRawButton(
			JoystickPorts::GO_BACKWARDS_REAR_LEFT_MOTOR)) {
		return BACKWARD;
	}
	else {
		return STILL;
	}
}

DS::Direction DS::lifterMotorDirection() {
	if (joystick_two->GetRawButton(JoystickPorts::GOING_UP_LIFTER)) {
		if (!joystick_two->GetRawButton(JoystickPorts::GOING_DOWN_LIFTER)) {
			return FORWARD;
		}
	}
	else if (joystick_two->GetRawButton(JoystickPorts::GOING_DOWN_LIFTER)) {
		return BACKWARD;
	}
	return STILL;
}

DS::Direction DS::armFlapMotorDirection() {
	if (joystick_one->GetRawButton(JoystickPorts::CLOSE_FLAPS)) {
		if (!joystick_one->GetRawButton(JoystickPorts::OPEN_FLAPS)) {
			return FORWARD;
		}
	}
	else if (joystick_one->GetRawButton(JoystickPorts::OPEN_FLAPS)) {
		return BACKWARD;
	}
	return STILL;
}

DS::Direction DS::portRakeMotorDirection() {
	if (joystick_two->GetRawButton(JoystickPorts::LIFT_PORT_RAKE)) {
		if (!joystick_two->GetRawButton(JoystickPorts::LOWER_PORT_RAKE)) {
			return FORWARD;
		}
	}
	else if (joystick_two->GetRawButton(JoystickPorts::LOWER_PORT_RAKE)) {
		return BACKWARD;
	}
	return STILL;
}

DS::Direction DS::starboardRakeMotorDirection() {
	if (joystick_two->GetRawButton(JoystickPorts::LIFT_STARBOARD_RAKE)) {
		if (!joystick_two->GetRawButton(JoystickPorts::LOWER_STARBOARD_RAKE)) {
			return FORWARD;
		}
	}
	else if (joystick_two->GetRawButton(JoystickPorts::LOWER_STARBOARD_RAKE)) {
		return BACKWARD;
	}
	return STILL;
}

DS::Direction DS::leftToteWheel() {
	if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_LEFT_WHEEL)) {
		if (!joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_LEFT_WHEEL)) {
			return FORWARD;
		}
	}
	else if (joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_LEFT_WHEEL)) {
		return BACKWARD;
	}
	return STILL;
}

DS::Direction DS::rightToteWheel() {
	if (joystick_two->GetRawButton(JoystickPorts::PULL_IN_RIGHT_WHEEL)) {
		if (!joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_RIGHT_WHEEL)) {
			return FORWARD;
		}
	}
	else if (joystick_two->GetRawButton(JoystickPorts::PUSH_OUT_RIGHT_WHEEL)) {
		return BACKWARD;
	}
	return STILL;
}

DS* DS::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new DS();
	}
	return INSTANCE;
}
