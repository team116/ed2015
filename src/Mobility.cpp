#include "Mobility.h"
#include "Ports.h"
#include <RobotDrive.h>
#include <Gyro.h>
#include <AnalogInput.h>
#include <CANTalon.h>
#include "Log.h"

using namespace std;

Mobility* Mobility::INSTANCE = NULL;
const float Mobility::DEFAULT_SPEED = 0.5;
const float Mobility::MAX_SPEED = 0.9;
const float Mobility::MAX_ULTRASONIC_DISTANCE = 254.0;
const float Mobility::MAX_ULTRASONIC_VOLTAGE = 5.5;

Mobility::Mobility()//COMMIT NUMBER 100
{
	front_left_motor = new CANTalon(RobotPorts::FRONT_LEFT_MOTOR);
	front_right_motor = new CANTalon(RobotPorts::FRONT_RIGHT_MOTOR);
	rear_left_motor = new CANTalon(RobotPorts::REAR_LEFT_MOTOR);
	rear_right_motor = new CANTalon(RobotPorts::REAR_RIGHT_MOTOR);
	robot_drive = new RobotDrive(front_left_motor, rear_left_motor, front_right_motor, rear_right_motor);
	robot_drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	robot_drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	robot_drive->SetSafetyEnabled(false);
	log = Log::getInstance();
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
	float angle = gyro->GetAngle();
	float rate = gyro->GetRate();
	float min_rate = 45.0f;
	float max_rate = 345.0f;
	float min_rot_speed = 0.2;
	float max_rot_speed = 0.75;
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

void Mobility::balanceVoltages()
{

}

void Mobility::setDirection(float x, float y)//-1.0 through 1.0
{
	x_direction = x * fabs(x) * MAX_SPEED;
	y_direction = y * fabs(y) * MAX_SPEED;
}
void Mobility::setRotationSpeed(float rotation_)//-1.0 through 1.0
{
	rotation = rotation_ * fabs(rotation_) * MAX_SPEED;
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
Mobility* Mobility::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Mobility();
    }
    return INSTANCE;
}
