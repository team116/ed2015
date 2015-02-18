#ifndef MOBILITY_H_
#define MOBILITY_H_

#include <RobotDrive.h>
#include <Gyro.h>
#include <AnalogInput.h>
#include <CANTalon.h>
#include <Encoder.h>
#include <Timer.h>
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
	void useClosedLoop(bool use);
	void useRealOrientation(bool real);
	void flipOrientation();
	void setPID(float p, float i, float d);
	float getP();
	float getI();
	float getD();


private:
	Mobility();
	static Mobility* INSTANCE;

	static float P_VALUE;
	static float I_VALUE;
	static float D_VALUE;
	bool using_closed_loop;

	static const float DEFAULT_SPEED;
	static const float MAX_SPEED;
	static const float RAMP_RATE;
	static const float MAX_ULTRASONIC_DISTANCE;
	static const float MAX_ULTRASONIC_VOLTAGE;
	static const float ODOMETRY_INCHES_PER_PULSE;
	static const float MAX_VELOCITY;
	CANTalon* front_left_motor;
	CANTalon* front_right_motor;
	CANTalon* rear_left_motor;
	CANTalon* rear_right_motor;
	RobotDrive* robot_drive;
	AnalogInput* ultrasonic;
	Gyro* gyro;
	double fr_enc_pos;
	double fl_enc_pos;
	double rr_enc_pos;
	double rl_enc_pos;
	Timer* time;


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

	/*
	Timer* speed_timer;
	bool past_ramping;
	int start_pos;
	*/
};

#endif // MOBILITY_H_
