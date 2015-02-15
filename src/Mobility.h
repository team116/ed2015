#ifndef MOBILITY_H_
#define MOBILITY_H_

#include <RobotDrive.h>
#include <Gyro.h>
#include <AnalogInput.h>
#include <CANTalon.h>
#include <Encoder.h>
#include "Log.h"

class Mobility
{
public:
	static Mobility* getInstance();
	void process();
	void setDirection(float x, float y);
	void setRotationSpeed(float rotation);
	void setRotationDegrees(int degrees);
	float getUltrasonicDistance();
	void toggleFieldCentric();
	int getXEncoderDistance();
	int getYEncoderDistance();
	void resetXEncoderDistance();
	void resetYEncoderDistance();
	void useRealOrientation(bool real);
	void flipOrientation();

private:
	Mobility();
	static Mobility* INSTANCE;
	static const float DEFAULT_SPEED;
	static const float MAX_SPEED;
	static const float RAMP_RATE;
	static const float MAX_ULTRASONIC_DISTANCE;
	static const float MAX_ULTRASONIC_VOLTAGE;
	static const float ODOMETRY_INCHES_PER_PULSE;
	CANTalon* front_left_motor;
	CANTalon* front_right_motor;
	CANTalon* rear_left_motor;
	CANTalon* rear_right_motor;
	RobotDrive* robot_drive;
	AnalogInput* ultrasonic;
	Gyro* gyro;

	Encoder* odometry_wheel_x_encoder;
	Encoder* odometry_wheel_y_encoder;

	Log* log;

	bool real_orientation;

	bool field_centric;
	bool rotating_degrees;

	//Turn Degrees
	float start_degrees;
	int rotate_direction;
	float target_degrees;

	float x_direction;
	float y_direction;
	float rotation;
};

#endif // MOBILITY_H_
