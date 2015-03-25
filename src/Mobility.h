#ifndef MOBILITY_H_
#define MOBILITY_H_

#include <Timer.h>
#include <RobotDrive.h>
#include <Gyro.h>
#include <AnalogInput.h>
#include <CANTalon.h>
#include <Encoder.h>
#include "Log.h"
#include "I2CCompass.h"
#include "I2CGyro.h"
#include <BuiltInAccelerometer.h>

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
	float getXEncoderDistance();
	float getYEncoderDistance();
	bool getRotatingDegrees();
	void resetXEncoderDistance();
	void resetYEncoderDistance();
	void useRealOrientation(bool real);
	void flipOrientation();
	void useClosedLoop(bool use);
	bool isVelZero();
	void setControlMode(CANSpeedController::ControlMode);

private:
	Mobility();
	static Mobility* INSTANCE;
	static const float DEFAULT_SPEED;
	static const float MAX_SPEED;
	static const float RAMP_RATE;
	static const float MAX_ULTRASONIC_DISTANCE;
	static const float MAX_ULTRASONIC_VOLTAGE;
	static const float X_ODOMETRY_INCHES_PER_PULSE;
	static const float Y_ODOMETRY_INCHES_PER_PULSE;
	static const float MAX_VELOCITY;
	static const float VOLTS_PER_INCH;
	static float SPEED_P_VALUE;
	static float SPEED_I_VALUE;
	static float SPEED_D_VALUE;
	static int SPEED_IZONE;
	static float POSITION_P_VALUE;
	static float POSITION_I_VALUE;
	static float POSITION_D_VALUE;
	static int POSITION_IZONE;
	static const float POSITION_ZONE;
	bool using_closed_loop;
	CANTalon* front_left_motor;
	CANTalon* front_right_motor;
	CANTalon* rear_left_motor;
	CANTalon* rear_right_motor;
	RobotDrive* robot_drive;
	AnalogInput* ultrasonic;
	I2CGyro* gyro;

	Encoder* odometry_wheel_x_encoder;
	Encoder* odometry_wheel_y_encoder;

	Log* log;

	I2CCompass* compass;
	Timer* rotation_timer;
	float rotation_timeout;

	BuiltInAccelerometer* accel;

	Timer* turn_timer;

	CANTalon::ControlMode control_mode;

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
