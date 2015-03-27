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
#include <PIDSource.h>
#include <PIDOutput.h>
#include <PIDController.h>

using namespace std;

float Mobility::SPEED_P_VALUE = 0.8f;
float Mobility::SPEED_I_VALUE = 0.004f;
float Mobility::SPEED_D_VALUE = 0.0f;
int Mobility::SPEED_IZONE = 1000;//1000 for speed mode
float Mobility::POSITION_P_VALUE = 0.8f;
float Mobility::POSITION_I_VALUE = 0.001f;
float Mobility::POSITION_D_VALUE = 10.0f;
int Mobility::POSITION_IZONE = 50;
const float Mobility::POSITION_ZONE = 50.0;

const float Mobility::ROT_P_VALUE = 0.0f;
const float Mobility::ROT_I_VALUE = 0.000f;
const float Mobility::ROT_D_VALUE = 0.0f;
const int Mobility::ROT_Izone = 10.0;
const float Mobility::GYRO_V_PER_DEG_PER_SEC = 0.007;
const float Mobility::GYRO_V_DEADZONE = 2.5;
const float Mobility::ROT_PID_MIN_IN = 0;
const float Mobility::ROT_PID_MAX_IN = 360;
const float Mobility::ROT_PID_MIN_OUT = 1.0;
const float Mobility::ROT_PID_MAX_OUT = -1.0;

Mobility* Mobility::INSTANCE = NULL;
const float Mobility::DEFAULT_SPEED = 0.5;
const float Mobility::MAX_SPEED = 0.9;
// measured in volts, ramps to full speed in 0.5 seconds
const float Mobility::RAMP_RATE = 24.0;
const float Mobility::MAX_ULTRASONIC_DISTANCE = 254.0;
const float Mobility::MAX_ULTRASONIC_VOLTAGE = 5.5;
const float Mobility::X_ODOMETRY_INCHES_PER_PULSE = 3.0 / 360.0;
const float Mobility::Y_ODOMETRY_INCHES_PER_PULSE = 3.0 / 250.0;
//const float Mobility::MAX_VELOCITY = 555.1f;
const float Mobility::MAX_VELOCITY = 750; //SOFTWARE BOT
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

	control_mode = CANTalon::kPercentVbus;
	setControlMode(CANTalon::kSpeed);

	robot_drive->SetSafetyEnabled(false);

	start_degrees = 0;
	rotate_direction = 0;
	target_degrees = 0;
	field_centric = false;
	rotating_degrees = false;
	//ultrasonic = new AnalogInput(RobotPorts::ULTRASONIC);
	//ultrasonic->SetOversampleBits(2);
	gyro = new Gyro(RobotPorts::GYRO);
	//gyro = I2CGyro::getInstance();
	gyro->SetSensitivity(GYRO_V_PER_DEG_PER_SEC);
	gyro->SetPIDSourceParameter(PIDSource::kAngle);
	pid_controller = new PIDController(ROT_P_VALUE, ROT_I_VALUE, ROT_D_VALUE, gyro, this);
	pid_controller->SetInputRange(ROT_PID_MIN_IN, ROT_PID_MAX_IN);
	pid_controller->SetOutputRange(ROT_PID_MIN_OUT, ROT_PID_MAX_OUT);
	pid_controller->SetContinuous();
	rotClosedLoop(false);

	x_direction = 0.0;
	y_direction = 0.0;
	rotation = 0.0;

	turn_timer = new Timer();

	log->write(Log::ERROR_LEVEL, "%s\tFinished Mobility construction\n", Utils::getCurrentTime());
}

void Mobility::process()
{
/*	POSITION_P_VALUE = std::stof(SmartDashboard::GetString("DB/String 0", std::to_string(POSITION_P_VALUE)));
	POSITION_I_VALUE = std::stof(SmartDashboard::GetString("DB/String 1", std::to_string(POSITION_I_VALUE)));
	POSITION_D_VALUE = std::stof(SmartDashboard::GetString("DB/String 2", std::to_string(POSITION_D_VALUE)));
	POSITION_IZONE = std::stoi(SmartDashboard::GetString("DB/String 3", std::to_string(POSITION_IZONE)));*/

	float angle = gyro->PIDGet();
	log->write(Log::TRACE_LEVEL, "%s\tGyro angle: %f", Utils::getCurrentTime(), angle);

	log->write(Log::TRACE_LEVEL, "%s\tOdometry wheel x raw: %d\n", Utils::getCurrentTime(), odometry_wheel_x_encoder->GetRaw());
	log->write(Log::TRACE_LEVEL, "%s\tOdometry wheel x dis: %f\n", Utils::getCurrentTime(), odometry_wheel_x_encoder->GetDistance());
	log->write(Log::TRACE_LEVEL, "%s\tOdometry wheel y raw: %d\n", Utils::getCurrentTime(), odometry_wheel_y_encoder->GetRaw());
	log->write(Log::TRACE_LEVEL, "%s\tOdometry wheel y dis: %f\n", Utils::getCurrentTime(), odometry_wheel_y_encoder->GetDistance());

	log->write(Log::TRACE_LEVEL, "%s\tFront right wheel velocity: %f\n", Utils::getCurrentTime(), front_right_motor->GetEncVel());
	log->write(Log::TRACE_LEVEL, "%s\tFront left wheel velocity: %f\n", Utils::getCurrentTime(), front_left_motor->GetEncVel());

	if (pid_controller->IsEnabled()) {
		if (rotation_timer->Get() > rotation_timeout) {
			// set our current angle as the goal
			rotation_timer->Stop();
			pid_controller->SetSetpoint(gyro->GetAngle());
			//pid_controller->SetSetpoint(gyro->getAngle());
			pid_controller->Reset();
			pid_controller->Enable();
		}

		// implement izone
		if (fabs(pid_controller->GetError()) < ROT_Izone) {
			pid_controller->Reset();
			pid_controller->Enable();
		}
	}
	else {
		if (rotating_degrees) {
			float rate = gyro->GetRate();
			//float rate = gyro->getRate();
			float min_rate = 45.0f;
			float max_rate = 345.0f;
			float min_rot_speed = 0.2;
			float max_rot_speed = 0.75;
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

			accel = 0.072f * (min((target_degrees - angle) / (angle - start_degrees), 1.0f) - (rate / max_rate));
			log->write(Log::ERROR_LEVEL, "%s\tRotation Speed: %f\n", Utils::getCurrentTime(), rotation);
			log->write(Log::ERROR_LEVEL, "%s\tRotation Difference: %f\n", Utils::getCurrentTime(), accel);
			// setRotationSpeed((float)rotate_direction * max(min(rotation + accel, max_rot_speed), min_rot_speed));
			rotation = (float)rotate_direction * max(min(rotation + accel, max_rot_speed), min_rot_speed);
		}
	}

	if(control_mode == CANSpeedController::kSpeed || control_mode == CANSpeedController::kPercentVbus) {
		log->write(Log::INFO_LEVEL, "%s\tMobility X: %f\n", Utils::getCurrentTime(), x_direction);
		log->write(Log::INFO_LEVEL, "%s\tMobility Y: %f\n", Utils::getCurrentTime(), y_direction);
		log->write(Log::INFO_LEVEL, "%s\tMobility R: %f\n", Utils::getCurrentTime(), rotation);
		if (field_centric) {
			robot_drive->MecanumDrive_Cartesian(x_direction, y_direction, rotation, angle);
		}
		else {
			robot_drive->MecanumDrive_Cartesian(x_direction, y_direction, rotation);
		}
	}
	else if(control_mode == CANSpeedController::kPosition){
		float fl_pos = front_left_motor->GetPosition();
		float fr_pos = front_right_motor->GetPosition();
		float rl_pos = rear_left_motor->GetPosition();
		float rr_pos = rear_right_motor->GetPosition();
//		front_left_motor->SetPosition((fl_pos / fabs(fl_pos)) * fabs(rl_pos));
//		front_right_motor->SetPosition((fr_pos / fabs(fr_pos)) * fabs(rr_pos));

		float fl_target = 0.0;
		float fr_target = 0.0;
		float rl_target = 0.0;
		float rr_target = 0.0;

		if(x_direction == 0.0 && y_direction != 0.0) {
			fl_target = -y_direction;
			fr_target = y_direction;
			rl_target = -y_direction;
			rr_target = y_direction;
		}
		else if(x_direction != 0.0 && y_direction == 0.0) {
			fl_target = -x_direction;
			fr_target = -x_direction;
			rl_target = x_direction;
			rr_target = x_direction;
		}

		front_left_motor->Set(fl_target);
		front_right_motor->Set(fr_target);
		rear_left_motor->Set(rl_target);
		rear_right_motor->Set(rr_target);

/*		log->write(Log::INFO_LEVEL, "RL target: %f RL pos: %f\n", rl_target, rl_pos);
		log->write(Log::INFO_LEVEL, "fabs: %f\n", fabs(rl_pos - rl_target));
//		log->write(Log::INFO_LEVEL, "RL pos: %f target: %f error: %d\n", rear_left_motor->GetPosition(), rear_left_motor->Get(), rear_left_motor->GetClosedLoopError());
		if(fabs(fl_pos - fl_target) <= POSITION_ZONE) {
			front_left_motor->SetPosition(fl_target);
			front_left_motor->SetControlMode(CANTalon::kPercentVbus);
			front_left_motor->Set(0.0);
			front_left_motor->SetControlMode(CANTalon::kPosition);
		}
		if(fabs(fr_pos - fr_target) <= POSITION_ZONE) {
			front_right_motor->SetPosition(fr_target);
			front_right_motor->SetControlMode(CANTalon::kPercentVbus);
			front_right_motor->Set(0.0);
			front_right_motor->SetControlMode(CANTalon::kPosition);
		}
		if(fabs(rl_pos - rl_target) <= POSITION_ZONE) {
			log->write(Log::INFO_LEVEL, "Seting rl position to target\n");
			rear_left_motor->SetPosition(rl_target);
			rear_left_motor->SetControlMode(CANTalon::kPercentVbus);
			rear_left_motor->Set(0.0);
			rear_left_motor->SetControlMode(CANTalon::kPosition);
		}
		if(fabs(rr_pos - rr_target) <= POSITION_ZONE) {
			rear_right_motor->SetPosition(rr_target);
			rear_right_motor->SetControlMode(CANTalon::kPercentVbus);
			rear_right_motor->Set(0.0);
			rear_right_motor->SetControlMode(CANTalon::kPosition);
		}*/
	}
}

void Mobility::setDirection(float x, float y)//-1.0 through 1.0
{
	x_direction = x * MAX_SPEED;
	y_direction = -y * MAX_SPEED;
	if (x_direction == 0.0 && y_direction == 0.0 && control_mode != CANTalon::kPercentVbus) {
		front_right_motor->PIDWrite(0.0);
		front_left_motor->PIDWrite(0.0);
		rear_right_motor->PIDWrite(0.0);
		rear_left_motor->PIDWrite(0.0);
	}
}

void Mobility::setRotationSpeed(float rotation_)//-1.0 through 1.0
{
	rotation = rotation_ * MAX_SPEED;
}

bool Mobility::isVelZero()
{
	int vel = 5;
	if(abs(front_left_motor->GetEncVel()) <= vel && abs(front_right_motor->GetEncVel()) <= vel &&
			abs(rear_left_motor->GetEncVel()) <= vel && abs(rear_right_motor->GetEncVel()) <= vel) {
		return true;
	}
	return false;
}

void Mobility::toggleFieldCentric()
{
	log->write(Log::TRACE_LEVEL, "%s\tToggling field centric\n", Utils::getCurrentTime());
	field_centric = !field_centric;
}

float Mobility::getUltrasonicDistance()
{
	// IDK how this is supposed to actually be but I'm going with this
	//float distance = ultrasonic->GetVoltage() / VOLTS_PER_INCH;
	return 0.0;
}

void Mobility::setRotationDegrees(int degrees)
{
	start_degrees = gyro->GetAngle();
	//start_degrees = gyro->getAngle();
	target_degrees = start_degrees + degrees;
	log->write(Log::ERROR_LEVEL, "%s\tStart Degrees: %f\n", Utils::getCurrentTime(), start_degrees);


	rotate_direction = 0;
	if (degrees > 0) {
		rotate_direction = 1;
	}
	else if (degrees < 0) {
		rotate_direction = -1;
	}

	if (pid_controller->IsEnabled()) {
		pid_controller->SetSetpoint(target_degrees);
	}
	else {
		rotating_degrees = true;
		setRotationSpeed(rotate_direction);
	}

	rotation_timeout = (float)degrees * 0.75 / 90.0; // estimating 0.75 seconds for every 90 degrees
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

/*void Mobility::useClosedLoop(bool use)
{
	// don't want to have to worry about unnecessary talon down time from switching configuration
	if (use != drive_closed_loop) {
		drive_closed_loop = use;
		if (use) {
			front_left_motor->SetControlMode(control_mode);
			front_left_motor->Set(0.0f);

			front_right_motor->SetControlMode(control_mode);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetControlMode(control_mode);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetControlMode(control_mode);
			rear_right_motor->Set(0.0f);

			switch(control_mode) {
			case CANTalon::kSpeed:
				log->write(Log::TRACE_LEVEL, "%s\tClosed Loop Speed Mode\n", Utils::getCurrentTime());

				front_left_motor->SetPID(SPEED_P_VALUE, SPEED_I_VALUE, SPEED_D_VALUE);
				front_left_motor->Set(0.0f);
				front_left_motor->SetIzone(SPEED_IZONE);
				front_left_motor->Set(0.0f);

				front_right_motor->SetPID(SPEED_P_VALUE, SPEED_I_VALUE, SPEED_D_VALUE);
				front_right_motor->Set(0.0f);
				front_right_motor->SetIzone(SPEED_IZONE);
				front_right_motor->Set(0.0f);

				rear_left_motor->SetPID(SPEED_P_VALUE, SPEED_I_VALUE, SPEED_D_VALUE);
				rear_left_motor->Set(0.0f);
				rear_left_motor->SetIzone(SPEED_IZONE);
				rear_left_motor->Set(0.0f);

				rear_right_motor->SetPID(SPEED_P_VALUE, SPEED_I_VALUE, SPEED_D_VALUE);
				rear_right_motor->Set(0.0f);
				rear_right_motor->SetIzone(SPEED_IZONE);
				rear_right_motor->Set(0.0f);

				robot_drive->SetMaxOutput(MAX_VELOCITY);
				break;
			case CANTalon::kPosition:
				log->write(Log::TRACE_LEVEL, "%s\tClosed Loop Position Mode\n", Utils::getCurrentTime());

				front_left_motor->SetPID(POSITION_P_VALUE, POSITION_I_VALUE, POSITION_D_VALUE);
				front_left_motor->Set(0.0f);
				front_left_motor->SetIzone(POSITION_IZONE);
				front_left_motor->Set(0.0f);

				front_right_motor->SetPID(POSITION_P_VALUE, POSITION_I_VALUE, POSITION_D_VALUE);
				front_right_motor->Set(0.0f);
				front_right_motor->SetIzone(POSITION_IZONE);
				front_right_motor->Set(0.0f);

				rear_left_motor->SetPID(POSITION_P_VALUE, POSITION_I_VALUE, POSITION_D_VALUE);
				rear_left_motor->Set(0.0f);
				rear_left_motor->SetIzone(POSITION_IZONE);
				rear_left_motor->Set(0.0f);

				rear_right_motor->SetPID(POSITION_P_VALUE, POSITION_I_VALUE, POSITION_D_VALUE);
				rear_right_motor->Set(0.0f);
				rear_right_motor->SetIzone(POSITION_IZONE);
				rear_right_motor->Set(0.0f);

				robot_drive->SetMaxOutput(1.0);
				break;
			}
		}
		else {
			log->write(Log::TRACE_LEVEL, "%s\tOpen Loop\n", Utils::getCurrentTime());

			control_mode = CANTalon::kPercentVbus;

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
			log->write(Log::TRACE_LEVEL, "%s\tOpen loop initialization finished\n", Utils::getCurrentTime());
		}
	}
}*/

void Mobility::setControlMode(CANSpeedController::ControlMode mode)
{
	if(control_mode != mode) {
		control_mode = mode;

		front_left_motor->SetControlMode(control_mode);
		front_left_motor->Set(0.0f);

		front_right_motor->SetControlMode(control_mode);
		front_right_motor->Set(0.0f);

		rear_left_motor->SetControlMode(control_mode);
		rear_left_motor->Set(0.0f);

		rear_right_motor->SetControlMode(control_mode);
		rear_right_motor->Set(0.0f);

		switch(mode) {
		case CANTalon::kSpeed:
			log->write(Log::INFO_LEVEL, "%s\tSetting control mode to speed\n", Utils::getCurrentTime());

			front_left_motor->SetPID(SPEED_P_VALUE, SPEED_I_VALUE, SPEED_D_VALUE);
			front_left_motor->SetIzone(SPEED_IZONE);
			front_left_motor->Set(0.0f);

			front_right_motor->SetPID(SPEED_P_VALUE, SPEED_I_VALUE, SPEED_D_VALUE);
			front_right_motor->SetIzone(SPEED_IZONE);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetPID(SPEED_P_VALUE, SPEED_I_VALUE, SPEED_D_VALUE);
			rear_left_motor->SetIzone(SPEED_IZONE);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetPID(SPEED_P_VALUE, SPEED_I_VALUE, SPEED_D_VALUE);
			rear_right_motor->SetIzone(SPEED_IZONE);
			rear_right_motor->Set(0.0f);

			robot_drive->SetMaxOutput(MAX_VELOCITY);
			break;
		case CANTalon::kPosition:
			log->write(Log::INFO_LEVEL, "%s\tSetting control mode to position\n", Utils::getCurrentTime());

			front_left_motor->SetPosition(0.0);
			front_right_motor->SetPosition(0.0);
			rear_left_motor->SetPosition(0.0);
			rear_right_motor->SetPosition(0.0);

			front_left_motor->SetPID(POSITION_P_VALUE, POSITION_I_VALUE, POSITION_D_VALUE);
			front_left_motor->SetIzone(POSITION_IZONE);
			front_left_motor->Set(0.0f);

			front_right_motor->SetPID(POSITION_P_VALUE, POSITION_I_VALUE, POSITION_D_VALUE);
			front_right_motor->SetIzone(POSITION_IZONE);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetPID(POSITION_P_VALUE, POSITION_I_VALUE, POSITION_D_VALUE);
			rear_left_motor->SetIzone(POSITION_IZONE);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetPID(POSITION_P_VALUE, POSITION_I_VALUE, POSITION_D_VALUE);
			rear_right_motor->SetIzone(POSITION_IZONE);
			rear_right_motor->Set(0.0f);

			robot_drive->SetMaxOutput(1.0);
			break;
		case CANTalon::kPercentVbus:
			log->write(Log::INFO_LEVEL, "%s\tSetting control mode to throttle\n", Utils::getCurrentTime());

			front_left_motor->SetPID(0.0, 0.0, 0.0);
			front_left_motor->SetIzone(0.0);
			front_left_motor->Set(0.0f);

			front_right_motor->SetPID(0.0, 0.0, 0.0);
			front_right_motor->SetIzone(0.0);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetPID(0.0, 0.0, 0.0);
			rear_left_motor->SetIzone(0.0);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetPID(0.0, 0.0, 0.0);
			rear_right_motor->SetIzone(0.0);
			rear_right_motor->Set(0.0f);

			robot_drive->SetMaxOutput(1.0);
			break;
		default:
			log->write(Log::WARNING_LEVEL, "%s\tWARNING: Unsupported control mode set: %d in Mobility.cpp at line"
					" number %d, defaulting to throttle\n", Utils::getCurrentTime(), mode, __LINE__);

			control_mode = CANTalon::kPercentVbus;

			front_left_motor->SetControlMode(control_mode);
			front_left_motor->Set(0.0f);

			front_right_motor->SetControlMode(control_mode);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetControlMode(control_mode);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetControlMode(control_mode);
			rear_right_motor->Set(0.0f);

			front_left_motor->SetPID(0.0, 0.0, 0.0);
			front_left_motor->SetIzone(0.0);
			front_left_motor->Set(0.0f);

			front_right_motor->SetPID(0.0, 0.0, 0.0);
			front_right_motor->SetIzone(0.0);
			front_right_motor->Set(0.0f);

			rear_left_motor->SetPID(0.0, 0.0, 0.0);
			rear_left_motor->SetIzone(0.0);
			rear_left_motor->Set(0.0f);

			rear_right_motor->SetPID(0.0, 0.0, 0.0);
			rear_right_motor->SetIzone(0.0);
			rear_right_motor->Set(0.0f);

			robot_drive->SetMaxOutput(1.0);
			break;
		}
	}
}

void Mobility::rotClosedLoop(bool rot)
{
	if (rot && !pid_controller->IsEnabled()) {
		pid_controller->Enable();
	}
	else if (pid_controller->IsEnabled()) {
		// the would mean that upon re-enable the robot might immediately start turning
		// pid_controller->Disable();

		// resetting the controller prevents that behavior, this also disables
		pid_controller->Reset();
	}
}

void Mobility::PIDWrite(float rot_speed)
{
	if (pid_controller->IsEnabled()) {
		// pid rotation disabled
		// setRotationSpeed(rot_speed);
	}
}

Mobility* Mobility::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Mobility();
    }
    return INSTANCE;
}
