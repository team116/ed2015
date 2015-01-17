#include "WPILib.h"
#include "Ports.h"
#include "Mobility.h"
#include "Gyro.h"
#include "TalonSRX.h"

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

Mobility* Mobility::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Mobility();
    }
    return INSTANCE;
}
