#include "WPILib.h"
#include "Ports.h"
#include "Mobility.h"
#include "Gyro.h"
#include "Victor.h"

Mobility* Mobility::INSTANCE = NULL;

Mobility::Mobility()
{
	front_left_motor = new TalonSRX(RobotPorts::FRONT_LEFT_MOTOR);
	front_right_motor = new TalonSRX(RobotPorts::FRONT_RIGHT_MOTOR);
	rear_left_motor = new TalonSRX(RobotPorts::REAR_LEFT_MOTOR);
	rear_right_motor = new TalonSRX(RobotPorts::REAR_RIGHT_MOTOR);
	robot_drive = new RobotDrive(front_left_motor, rear_left_motor, front_right_motor, rear_right_motor);
	x_direction = 0;
	y_direction = 0;
	rotation = 0;
	ultrasonic = new AnalogInput(RobotPorts::ULTRASONIC);
}

void Mobility::process()
{
	robot_drive->MecanumDrive_Cartesian(x_direction, y_direction, rotation);
}

void Mobility::setDirection(float x, float y)//-1.0 through 1.0
{
	x_direction = x;
	y_direction = y;
}
void Mobility::setRotation(float rotation_)//-1.0 through 1.0
{
	rotation = rotation_;
}
double Mobility::getUltrasonicDistance()
{
	int bits;
	double maxDistance = 254;
	double currentDistance;
	double maxVoltage = 5.5;
	// sets roof for sampling values
	ultrasonic->SetOversampleBits(2);
	bits = ultrasonic->GetOversampleBits();
	// number that 2^ that the number of samples is reduced by...
	ultrasonic->SetAverageBits(1);
	bits = ultrasonic->GetAverageBits();

	ultrasonic->SetSampleRate(62500);
	int raw = ultrasonic->GetValue();
	double volts = ultrasonic->GetVoltage();
	int averageRaw = ultrasonic->GetAverageValue();
	double averageVolts = ultrasonic->GetAverageVoltage();
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
