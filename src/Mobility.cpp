#include "Mobility.h"
#include "Ports.h"
#include <RobotDrive.h>
#include <Gyro.h>
#include <AnalogInput.h>
#include <CANTalon.h>
#include <CANSpeedController.h>
#include <Timer.h>
#include "Log.h"

using namespace std;

Mobility* Mobility::INSTANCE = NULL;

const float Mobility::P_VALUE = 0.9f;
const float Mobility::I_VALUE = 0.0f;
const float Mobility::D_VALUE = 3.0f;

const float Mobility::DEFAULT_SPEED = 0.5f;
const float Mobility::MAX_SPEED = 0.9f;
const float Mobility::RAMP_RATE = 24.0f; // measured in volts, ramps to full speed in 0.5 seconds
const float Mobility::MAX_ULTRASONIC_DISTANCE = 254.0f;
const float Mobility::MAX_ULTRASONIC_VOLTAGE = 5.5f;
const float Mobility::ODOMETRY_INCHES_PER_PULSE = 3.0f/360.0f;
const float Mobility::MAX_VELOCITY = 1750.0f;

Mobility::Mobility()//COMMIT NUMBER 100
{
	log = Log::getInstance();
	front_left_motor = new CANTalon(RobotPorts::FRONT_LEFT_MOTOR);
	front_left_motor->SetVoltageRampRate(RAMP_RATE);
	front_left_motor->Set(0.0);
	front_left_motor->SetFeedbackDevice(CANTalon::QuadEncoder);
	// front_left_motor->SetControlMode(CANTalon::kSpeed);

	front_right_motor = new CANTalon(RobotPorts::FRONT_RIGHT_MOTOR);
	front_right_motor->SetVoltageRampRate(RAMP_RATE);
	front_right_motor->Set(0.0);
	front_right_motor->SetFeedbackDevice(CANTalon::QuadEncoder);
	// front_right_motor->SetControlMode(CANTalon::kSpeed);

	// speed_timer = new Timer();
	// past_ramping = false;
	// start_pos = 0;
	rear_left_motor = new CANTalon(RobotPorts::REAR_LEFT_MOTOR);
	rear_left_motor->SetVoltageRampRate(RAMP_RATE);
	rear_left_motor->Set(0.0);
	rear_left_motor->SetFeedbackDevice(CANTalon::QuadEncoder);
//	rear_left_motor->SetPosition(0.0);
	// speed_timer->Start();
	// rear_left_motor->Set(0.9);
//	rear_left_motor->SetControlMode(CANTalon::kSpeed);
	// rear_left_motor->SetControlMode(CANTalon::kPosition);
//	rear_left_motor->SetPID(0.9, 0.0, 0.0, 0.0);
	// rear_left_motor->SetIzone(0);
	rear_left_motor->SetSensorDirection(false);
	// rear_left_motor->SetCloseLoopRampRate(12.0);
	// rear_left_motor->ClearError();
	// rear_left_motor->ClearIaccum();
	// rear_left_motor->Set(1.0);
//	rear_left_motor->Set(0.0);

	rear_right_motor = new CANTalon(RobotPorts::REAR_RIGHT_MOTOR);
	rear_right_motor->SetVoltageRampRate(RAMP_RATE);
	rear_right_motor->Set(0.0);
	rear_right_motor->SetFeedbackDevice(CANTalon::QuadEncoder);
	// rear_right_motor->SetControlMode(CANTalon::kSpeed);

	odometry_wheel_x_encoder = new Encoder(RobotPorts::ODOMETRY_WHEEL_X_A,RobotPorts::ODOMETRY_WHEEL_X_B);
	odometry_wheel_y_encoder = new Encoder(RobotPorts::ODOMETRY_WHEEL_Y_A,RobotPorts::ODOMETRY_WHEEL_Y_B);

	odometry_wheel_x_encoder->SetDistancePerPulse(ODOMETRY_INCHES_PER_PULSE);
	odometry_wheel_y_encoder->SetDistancePerPulse(ODOMETRY_INCHES_PER_PULSE);

	robot_drive = new RobotDrive(front_left_motor, rear_left_motor, front_right_motor, rear_right_motor);
	// set to true is this is the software bot
	real_orientation = true;
	useRealOrientation(real_orientation);
	//if (false) {
	//	robot_drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	//	robot_drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	//}
	//else {
	//	robot_drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
	//	robot_drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	//}
	robot_drive->SetSafetyEnabled(false);

	// closed loop initialization, change this to false if we don't want to default to closed loop
	using_closed_loop = false;
	useClosedLoop(true);
/*	if (using_closed_loop) {
			front_left_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			front_left_motor->SetControlMode(CANTalon::kSpeed);
			front_left_motor->Set(0.0f);

			front_right_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			front_right_motor->SetControlMode(CANTalon::kSpeed);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			rear_left_motor->SetControlMode(CANTalon::kSpeed);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			rear_right_motor->SetControlMode(CANTalon::kSpeed);
			rear_right_motor->Set(0.0f);

			robot_drive->SetMaxOutput(MAX_VELOCITY);
			rear_left_motor->Set(0.5 * MAX_VELOCITY);
			rear_right_motor->Set(-0.5 * MAX_VELOCITY);
	}
	else {
		front_left_motor->SetPID(0.0, 0.0, 0.0);
		front_left_motor->SetControlMode(CANTalon::kPercentVbus);

		front_right_motor->SetPID(0.0, 0.0, 0.0);
		front_right_motor->SetControlMode(CANTalon::kPercentVbus);

		rear_left_motor->SetPID(0.0, 0.0, 0.0);
		rear_left_motor->SetControlMode(CANTalon::kPercentVbus);

		rear_right_motor->SetPID(0.0, 0.0, 0.0);
		rear_right_motor->SetControlMode(CANTalon::kPercentVbus);

		robot_drive->SetMaxOutput(1.0);
		rear_left_motor->Set(0.5);
		rear_right_motor->Set(-0.5);
	}*/

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
	gyro = new Gyro(RobotPorts::GYRO);
}

void Mobility::process()
{
	/*
	rear_left_motor->Set(0.9);
	if (speed_timer->Get() > 5.0)
	{
		log->write(Log::INFO_LEVEL, "%s\tYO DAWG, max velocity: %f\n", Utils::getCurrentTime(), (float)(rear_left_motor->GetEncPosition() - start_pos) / speed_timer->Get());
		start_pos = rear_left_motor->GetEncPosition();
		speed_timer->Reset();
	}
	*/
	float angle = gyro->GetAngle();
	float rate = gyro->GetRate();
	float min_rate = 45.0f;
	float max_rate = 345.0f;
	float min_rot_speed = 0.2;
	float max_rot_speed = 0.75;
	log->write(Log::INFO_LEVEL, "Rate: %f\n", rate);
	//rear_left_motor->Set(0.0);
	//rear_right_motor->Set(0.0);
	//front_left_motor->Set(0.0);
	//front_right_motor->Set(0.0);

	// spam the logs...
	log->write(Log::INFO_LEVEL, "%s\tfront left encoder: %i\n", Utils::getCurrentTime(), front_left_motor->GetEncPosition());
	log->write(Log::INFO_LEVEL, "%s\tfront right encoder: %i\n", Utils::getCurrentTime(), front_right_motor->GetEncPosition());
	log->write(Log::INFO_LEVEL, "%s\trear left encoder: %i\n", Utils::getCurrentTime(), rear_left_motor->GetEncPosition());
	log->write(Log::INFO_LEVEL, "%s\trear right encoder: %i\n", Utils::getCurrentTime(), rear_right_motor->GetEncPosition());

	if(rotating_degrees)
	{
		log->write(Log::ERROR_LEVEL, "Gyro: %f\n", angle);
		if(((rotate_direction == 1) && (angle >= target_degrees)) || ((rotate_direction == -1) && (angle <= target_degrees))) {
			rotate_direction = 0;
		}
		if (rotate_direction == 0) {
			log->write(Log::ERROR_LEVEL, "Rotating finished\n");
			rotating_degrees = false;
		}
		// float var = ((((rate - min_rate)/(max_rate - min_rate))) - (max(min(fabs(target_degrees - angle),0.8f), 0.2f))) * 0.072f;
		// log->write(Log::ERROR_LEVEL, "Rotate Difference: %f\n", var);
		// log->write(Log::ERROR_LEVEL, "Rotation Speed: %f\n", rotation + var);
		// setRotationSpeed(rotation + var);
		float accel = 0.072f * (min((target_degrees - angle) / (angle - start_degrees), 1.0f) - (gyro->GetRate() / max_rate));
		log->write(Log::ERROR_LEVEL, "Rotation Speed: %f\n", rotation);
		log->write(Log::ERROR_LEVEL, "Rotation Difference: %f\n", accel);
		//setRotationSpeed((float)rotate_direction * max(min(rotation + accel, max_rot_speed), min_rot_speed));
		rotation = (float)rotate_direction * max(min(rotation + accel, max_rot_speed), min_rot_speed);
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
	log->write(Log::INFO_LEVEL, "Toggling field centric\n");
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
	//IDK how this is supposed to actually be but I'm going with this
	return (ultrasonic->GetVoltage() * MAX_ULTRASONIC_DISTANCE)/MAX_ULTRASONIC_VOLTAGE;
}

void Mobility::setRotationDegrees(int degrees)
{
	start_degrees = gyro->GetAngle();
	rotate_direction = 0;
	if (degrees > 0) {
		rotate_direction = 1;
	}
	else if (degrees < 0) {
		rotate_direction = -1;
	}
	target_degrees = start_degrees + degrees;
	log->write(Log::ERROR_LEVEL, "Start Degrees: %f\n", start_degrees);
	rotating_degrees = true;
	setRotationSpeed(rotate_direction);
}

void Mobility::resetXEncoderDistance(){
	odometry_wheel_x_encoder->Reset();
}

void Mobility::resetYEncoderDistance(){
	odometry_wheel_y_encoder->Reset();
}

int Mobility::getXEncoderDistance(){
	return odometry_wheel_x_encoder->GetDistance();
}

int Mobility::getYEncoderDistance(){
	return odometry_wheel_y_encoder->GetDistance();
}

void Mobility::useClosedLoop(bool use)
{
	// don't want to have to worry about unnecessary talon down time from switching configuration
	if (use != using_closed_loop) {
		using_closed_loop = use;
		if (use) {
			log->write(Log::INFO_LEVEL, "Closed Loop\n");
			front_left_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			front_left_motor->SetControlMode(CANTalon::kSpeed);
			front_left_motor->Set(0.0f);

			front_right_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			front_right_motor->SetControlMode(CANTalon::kSpeed);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			rear_left_motor->SetControlMode(CANTalon::kSpeed);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetPID(P_VALUE, I_VALUE, D_VALUE);
			rear_right_motor->SetControlMode(CANTalon::kSpeed);
			rear_right_motor->Set(0.0f);

			robot_drive->SetMaxOutput(MAX_VELOCITY);
			rear_left_motor->Set(0.0 * MAX_VELOCITY);
			rear_right_motor->Set(-0.0 * MAX_VELOCITY);
		}
		else {
			log->write(Log::INFO_LEVEL, "Open Loop\n");
			front_left_motor->SetPID(0.0, 0.0, 0.0);
			front_left_motor->SetControlMode(CANTalon::kPercentVbus);

			front_right_motor->SetPID(0.0, 0.0, 0.0);
			front_right_motor->SetControlMode(CANTalon::kPercentVbus);

			rear_left_motor->SetPID(0.0, 0.0, 0.0);
			rear_left_motor->SetControlMode(CANTalon::kPercentVbus);

			rear_right_motor->SetPID(0.0, 0.0, 0.0);
			rear_right_motor->SetControlMode(CANTalon::kPercentVbus);

			robot_drive->SetMaxOutput(1.0);
			rear_left_motor->Set(0.0);
			rear_right_motor->Set(-0.0);
		}
	}
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
	if (real_orientation) {
		useRealOrientation(false);
	}
	else {
		useRealOrientation(true);
	}
}

Mobility* Mobility::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Mobility();
    }
    return INSTANCE;
}
