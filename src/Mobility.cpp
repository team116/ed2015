#include "Mobility.h"
#include "Ports.h"
#include <RobotDrive.h>
#include <Gyro.h>
#include <AnalogInput.h>
#include <CANTalon.h>
#include <CANSpeedController.h>
#include "Log.h"
#include "I2CCompass.h"
#include "I2CGyro.h"
#include <BuiltInAccelerometer.h>

using namespace std;

float Mobility::P_VALUE = 0.9f;
float Mobility::I_VALUE = 0.1f;
float Mobility::D_VALUE = 0.0f;
Mobility* Mobility::INSTANCE = NULL;
const float Mobility::DEFAULT_SPEED = 0.5;
const float Mobility::MAX_SPEED = 0.9;
// measured in volts, ramps to full speed in 0.5 seconds
const float Mobility::RAMP_RATE = 24.0;
const float Mobility::MAX_ULTRASONIC_DISTANCE = 254.0;
const float Mobility::MAX_ULTRASONIC_VOLTAGE = 5.5;
const float Mobility::X_ODOMETRY_INCHES_PER_PULSE = 3.0 / 360.0;
const float Mobility::Y_ODOMETRY_INCHES_PER_PULSE = 3.0 / 250.0;
const float Mobility::MAX_VELOCITY = 555.1f;
const float Mobility::VOLTS_PER_INCH = 5.0 / 512.0;
// 250 PPR on drive wheel encoders


Mobility::Mobility()
{
	log = Log::getInstance();
	log->write(Log::DEBUG_LEVEL, "%s\tConstructing Mobility.cpp\n", Utils::getCurrentTime());

	front_left_motor = new CANTalon(RobotPorts::FRONT_LEFT_MOTOR);
	front_left_motor->SetVoltageRampRate(RAMP_RATE);
	front_left_motor->Set(0.0);
	front_left_motor->SetFeedbackDevice(CANTalon::QuadEncoder);
	front_left_motor->SetSensorDirection(true);
	front_left_motor->Set(0.0);

	front_right_motor = new CANTalon(RobotPorts::FRONT_RIGHT_MOTOR);
	front_right_motor->SetVoltageRampRate(RAMP_RATE);
	front_right_motor->Set(0.0);
	front_right_motor->SetFeedbackDevice(CANTalon::QuadEncoder);
	front_right_motor->SetSensorDirection(true);
	front_right_motor->Set(0.0);

	rear_left_motor = new CANTalon(RobotPorts::REAR_LEFT_MOTOR);
	rear_left_motor->SetVoltageRampRate(RAMP_RATE);
	rear_left_motor->Set(0.0);
	rear_left_motor->SetFeedbackDevice(CANTalon::QuadEncoder);
	rear_left_motor->SetSensorDirection(true);
	rear_left_motor->Set(0.0);

	rear_right_motor = new CANTalon(RobotPorts::REAR_RIGHT_MOTOR);
	rear_right_motor->SetVoltageRampRate(RAMP_RATE);
	rear_right_motor->Set(0.0);
	rear_right_motor->SetFeedbackDevice(CANTalon::QuadEncoder);
	rear_right_motor->SetSensorDirection(true);
	rear_right_motor->Set(0.0);

	odometry_wheel_x_encoder = new Encoder(RobotPorts::ODOMETRY_WHEEL_X_A,RobotPorts::ODOMETRY_WHEEL_X_B);
	odometry_wheel_y_encoder = new Encoder(RobotPorts::ODOMETRY_WHEEL_Y_A,RobotPorts::ODOMETRY_WHEEL_Y_B);

	odometry_wheel_x_encoder->SetDistancePerPulse(X_ODOMETRY_INCHES_PER_PULSE);
	odometry_wheel_y_encoder->SetDistancePerPulse(Y_ODOMETRY_INCHES_PER_PULSE);

	compass = I2CCompass::getInstance();
	rotation_timeout = 0.0;

	accel = new BuiltInAccelerometer(BuiltInAccelerometer::kRange_4G);

	robot_drive = new RobotDrive(front_left_motor, rear_left_motor, front_right_motor, rear_right_motor);

	// set to true if this is the software bot
	real_orientation = true;
	useRealOrientation(real_orientation);

	//SET THIS to the opposite of what you really want
	using_closed_loop = true;
	//DONT SET THIS
	useClosedLoop(!using_closed_loop);

	robot_drive->SetSafetyEnabled(false);
	x_direction = 0;
	y_direction = 0;
	rotation = 0;
	start_degrees = 0;
	rotate_direction = 0;
	target_degrees = 0;
	field_centric = false;
	rotating_degrees = false;
	ultrasonic = new AnalogInput(RobotPorts::ULTRASONIC);
	ultrasonic->SetOversampleBits(2);
	//gyro = new Gyro(RobotPorts::GYRO);
	gyro = I2CGyro::getInstance();

	turn_timer = new Timer();
}

void Mobility::process()
{
/*	POSITION_P_VALUE = std::stof(SmartDashboard::GetString("DB/String 0", std::to_string(POSITION_P_VALUE)));
	POSITION_I_VALUE = std::stof(SmartDashboard::GetString("DB/String 1", std::to_string(POSITION_I_VALUE)));
	POSITION_D_VALUE = std::stof(SmartDashboard::GetString("DB/String 2", std::to_string(POSITION_D_VALUE)));
	POSITION_IZONE = std::stoi(SmartDashboard::GetString("DB/String 3", std::to_string(POSITION_IZONE)));*/

	float angle = gyro->getAngle();
	float rate = gyro->getRate();
	float min_rate = 45.0f;
	float max_rate = 345.0f;
	float min_rot_speed = 0.2;
	float max_rot_speed = 0.75;

	if(rotating_degrees)
	{
		float accel = 0.0;
		log->write(Log::ERROR_LEVEL, "%s\tGyro: %f\n", Utils::getCurrentTime(), angle);
		if(((rotate_direction == 1) && (angle >= target_degrees)) ||
			((rotate_direction == -1) && (angle <= target_degrees)) ||
			turn_timer->Get() > rotation_timeout) {
			rotate_direction = 0;
		}
		else if(((rotate_direction == 1) && (angle >= target_degrees)) || ((rotate_direction == -1) && (angle <= target_degrees))) {
			rotate_direction = 0;
		}

		if (rotate_direction == 0) {
			log->write(Log::ERROR_LEVEL, "%s\tRotating finished\n", Utils::getCurrentTime());
			rotating_degrees = false;
			rotation_timeout = 0.0;
			rotation_timer->Stop();
		}
		// float var = ((((rate - min_rate)/(max_rate - min_rate))) - (max(min(fabs(target_degrees - angle),0.8f), 0.2f))) * 0.072f;
		// log->write(Log::ERROR_LEVEL, "Rotate Difference: %f\n", var);
		// log->write(Log::ERROR_LEVEL, "Rotation Speed: %f\n", rotation + var);
		// setRotationSpeed(rotation + var);
		/*
		accel = 0.072f * (min((target_degrees - angle) / (angle - start_degrees), 1.0f) - (rate / max_rate));
		log->write(Log::ERROR_LEVEL, "%s\tRotation Speed: %f\n", Utils::getCurrentTime(), rotation);
		log->write(Log::ERROR_LEVEL, "%s\tRotation Difference: %f\n", Utils::getCurrentTime(), accel);
		// setRotationSpeed((float)rotate_direction * max(min(rotation + accel, max_rot_speed), min_rot_speed));
		rotation = (float)rotate_direction * max(min(rotation + accel, max_rot_speed), min_rot_speed);
		*/
	}
	if (field_centric) {
		robot_drive->MecanumDrive_Cartesian(x_direction, y_direction, rotation, angle);
	}
	else {
		robot_drive->MecanumDrive_Cartesian(x_direction, y_direction, rotation);
	}
}

void Mobility::setDirection(float x, float y)//-1.0 through 1.0
{
	x_direction = x * MAX_SPEED;
	y_direction = y * MAX_SPEED;
}

void Mobility::setRotationSpeed(float rotation_)//-1.0 through 1.0
{
	rotation = rotation_ * MAX_SPEED;
}

void Mobility::toggleFieldCentric()
{
	log->write(Log::TRACE_LEVEL, "%s\tToggling field centric\n", Utils::getCurrentTime());
	field_centric = !field_centric;
}

float Mobility::getUltrasonicDistance()
{
 	/*
	int bits;
	float maxDistance = 254.0;
	float currentDistance;
	float maxVoltage = 5.5;
	// sets roof for sampling values
	ultrasonic->SetOversampleBits(2);
	bits = ultrasonic->GetOversampleBits();
	// number that 2^ that the number of samples is reduced by...
	ultrasonic->SetAverageBits(1);
	bits = ultrasonic->GetAverageBits();

	ultrasonic->SetSampleRate(62500);
	int raw = ultrasonic->GetValue();
	float volts = ultrasonic->GetVoltage();
	int averageRaw = ultrasonic->GetAverageValue();
	float averageVolts = ultrasonic->GetAverageVoltage();
	//	wait for iiiiiiiittt.....
	currentDistance = (volts * maxDistance)/maxVoltage;
	return currentDistance;
	*/
	// IDK how this is supposed to actually be but I'm going with this
	float distance = ultrasonic->GetVoltage() / VOLTS_PER_INCH;
	return distance;
}

void Mobility::setRotationDegrees(int degrees)
{
	start_degrees = gyro->getAngle();
	rotate_direction = 0;
	if (degrees > 0) {
		rotate_direction = 1;
	}
	else if (degrees < 0) {
		rotate_direction = -1;
	}
	target_degrees = start_degrees + degrees;
	log->write(Log::ERROR_LEVEL, "%s\tStart Degrees: %f\n", Utils::getCurrentTime(), start_degrees);
	rotating_degrees = true;
	rotation_timeout = (float)degrees * 0.75 / 90.0; // estimating 0.75 seconds for every 90 degrees
	setRotationSpeed(rotate_direction);
	turn_timer->Reset();
	turn_timer->Start();
}

void Mobility::resetXEncoderDistance()
{
	odometry_wheel_x_encoder->Reset();
}

void Mobility::resetYEncoderDistance()
{
	odometry_wheel_y_encoder->Reset();
}

float Mobility::getXEncoderDistance()
{
	return odometry_wheel_x_encoder->GetDistance();
}

float Mobility::getYEncoderDistance()
{
	return odometry_wheel_y_encoder->GetDistance();
}

bool Mobility::getRotatingDegrees()
{
	return rotating_degrees;
}

void Mobility::useRealOrientation(bool real)
{
	real_orientation = real;
	if (real) {
		robot_drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		robot_drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		robot_drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, false);
		robot_drive->SetInvertedMotor(RobotDrive::kRearRightMotor, false);
	}
	else {
		robot_drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);
		robot_drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, false);
		robot_drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		robot_drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	}
}

void Mobility::flipOrientation()
{
	useRealOrientation(!real_orientation);
}

void Mobility::useClosedLoop(bool use)
{
	// don't want to have to worry about unnecessary talon down time from switching configuration
	if (use != using_closed_loop) {
		using_closed_loop = use;
		if (use) {
			log->write(Log::TRACE_LEVEL, "Closed Loop\n");
			front_left_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			front_left_motor->Set(0.0f);
			front_left_motor->SetControlMode(CANTalon::kSpeed);
			front_left_motor->Set(0.0f);

			front_right_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			front_right_motor->Set(0.0f);
			front_right_motor->SetControlMode(CANTalon::kSpeed);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			rear_left_motor->Set(0.0f);
			rear_left_motor->SetControlMode(CANTalon::kSpeed);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			rear_right_motor->Set(0.0f);
			rear_right_motor->SetControlMode(CANTalon::kSpeed);
			rear_right_motor->Set(0.0f);

			robot_drive->SetMaxOutput(MAX_VELOCITY);
//			rear_left_motor->Set(205);
//			rear_right_motor->Set(-0.2 * MAX_VELOCITY);
//			front_left_motor->Set(0.2 * MAX_VELOCITY);
//			front_right_motor->Set(-0.2 * MAX_VELOCITY);
		}
		else {
			log->write(Log::TRACE_LEVEL, "Open Loop\n");
			front_left_motor->SetPID(0.0, 0.0, 0.0);
			front_left_motor->Set(0.0f);
			front_left_motor->SetControlMode(CANTalon::kPercentVbus);
			front_left_motor->Set(0.0f);

			front_right_motor->SetPID(0.0, 0.0, 0.0);
			front_right_motor->Set(0.0f);
			front_right_motor->SetControlMode(CANTalon::kPercentVbus);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetPID(0.0, 0.0, 0.0);
			rear_left_motor->Set(0.0f);
			rear_left_motor->SetControlMode(CANTalon::kPercentVbus);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetPID(0.0, 0.0, 0.0);
			rear_right_motor->Set(0.0f);
			rear_right_motor->SetControlMode(CANTalon::kPercentVbus);
			rear_right_motor->Set(0.0f);

			robot_drive->SetMaxOutput(1.0);
//			rear_left_motor->Set(0.2);
//			rear_right_motor->Set(-0.2);
//			front_left_motor->Set(0.2);
//			front_right_motor->Set(-0.2);
		}
	}
}

Mobility* Mobility::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Mobility();
    }
    return INSTANCE;
}
