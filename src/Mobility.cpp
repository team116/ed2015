#include <WPILib.h>
#include <Gyro.h>
#include <CANTalon.h>
#include "Mobility.h"
#include "Ports.h"
#include "Log.h"
#include <cmath>
#include <BuiltInAccelerometer.h>

Mobility* Mobility::INSTANCE = NULL;

Mobility::Mobility()
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
	field_centric = false;
	ultrasonic = new AnalogInput(RobotPorts::ULTRASONIC);
	gyro = new Gyro(RobotPorts::GYRO);
}

void Mobility::process()
{
	if(field_centric)
		robot_drive->MecanumDrive_Cartesian(x_direction, y_direction, rotation, gyro->GetAngle());
	else
		robot_drive->MecanumDrive_Cartesian(x_direction, y_direction, rotation);
}

void Mobility::setDirection(float x, float y)//-1.0 through 1.0
{
	x_direction = x * fabs(x) * 0.90;
	y_direction = y * fabs(y) * 0.90;
}
void Mobility::setRotation(float rotation_)//-1.0 through 1.0
{
	rotation = rotation_ * fabs(rotation_) * 0.90;
}
void Mobility::runTalon(int talon, float speed)//For testing individual talons. You MUST comment out robot_drive to use this
{
	log->write(Log::INFO_LEVEL, "Run Talon %d", talon);
	switch(talon)
	{
	case RobotPorts::FRONT_LEFT_MOTOR: front_left_motor->Set(speed); break;
	case RobotPorts::FRONT_RIGHT_MOTOR: front_right_motor->Set(speed); break;
	case RobotPorts::REAR_LEFT_MOTOR: rear_left_motor->Set(speed); break;
	case RobotPorts::REAR_RIGHT_MOTOR: rear_right_motor->Set(speed); break;
	}
}
void Mobility::toggleFieldCentric()
{
	log->write(Log::INFO_LEVEL, "Toggling field centric\n");
	field_centric = !field_centric;
}
float Mobility::getUltrasonicDistance()
{
	int bits;
	float maxDistance = 254;
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
}

Mobility* Mobility::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Mobility();
    }
    return INSTANCE;
}
