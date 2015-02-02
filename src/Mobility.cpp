#include <WPILib.h>
#include <Gyro.h>
#include <CANTalon.h>
#include "Mobility.h"
#include "Ports.h"
#include "Log.h"
#include <cmath>
#include <BuiltInAccelerometer.h>

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
	accel = new BuiltInAccelerometer();
	log = Log::getInstance();
	x_direction = 0;
	y_direction = 0;
	rotation = 0;
	start_degrees = 0;
	rotate_degrees = 0;
	field_centric = false;
	rotating_degrees = false;
	ultrasonic = new AnalogInput(RobotPorts::ULTRASONIC);
	ultrasonic->SetOversampleBits(2);
	gyro = new Gyro(RobotPorts::GYRO);
}

void Mobility::process()
{
	if(rotating_degrees)
	{
		if((int)gyro->GetAngle() == (start_degrees + rotate_degrees))
		{
			setRotationSpeed(0);
			rotating_degrees = false;
			rotate_degrees = 0;
			start_degrees = 0;
		}
		else
		{
			if(rotate_degrees < 0)
				setRotationSpeed(-DEFAULT_SPEED);
			else if(rotate_degrees > 0)
				setRotationSpeed(DEFAULT_SPEED);
			else
				setRotationSpeed(0.0);
		}
	}
	if(field_centric)
		robot_drive->MecanumDrive_Cartesian(x_direction, y_direction, rotation, gyro->GetAngle());
	else
		robot_drive->MecanumDrive_Cartesian(x_direction, y_direction, rotation);
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
	rotating_degrees = true;
}
Mobility* Mobility::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Mobility();
    }
    return INSTANCE;
}
