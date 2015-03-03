/*
 * FieldMapping.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Will
 */

#include "FieldMapping.h"
#include "Mobility.h"
#include <BuiltInAccelerometer.h>
#include <cmath>

FieldMapping* FieldMapping::INSTANCE = NULL;

//TODO:Get actual robot dimensions
const float FieldMapping::ROBOT_WIDTH = 0.0;
const float FieldMapping::ROBOT_LENGTH = 0.0;

FieldMapping::FieldMapping()
{
	accel = new BuiltInAccelerometer(BuiltInAccelerometer::kRange_4G);
	gyro = I2CGyro::getInstance();

	//Your starting position
	GetCoordinates(STAGING_1, excuse_me_question_mark);

	//in inches
	x_pos = excuse_me_question_mark[0];
	y_pos = excuse_me_question_mark[1];

	angle = 0.0;
	x_velocity = 0.0;
	y_velocity = 0.0;

	velocity_timer = new Timer();
	velocity_timer->Start();
}

void FieldMapping::process()
{
	float distance = 0.0;
	float robot_y_distance = 0.0;
	float robot_x_distance = 0.0;

	float robot_angle_travelled = 0.0;
	float field_angle_travelled = 0.0;
	angle = gyro->getAngle();

	float time_passed = velocity_timer->Get();
	//1 g equals 386.0885826673 inches per second per second
	float x_accel = accel->GetX() * 386.0885826673 * time_passed;
	float y_accel = accel->GetY() * 386.0885826673 * time_passed;
	x_velocity += x_accel;
	y_velocity += y_accel;

	robot_x_distance = x_velocity * time_passed;
	robot_y_distance = y_velocity * time_passed;
	distance += sqrt((x_accel * x_accel) + (y_accel * y_accel)) * time_passed;
	robot_angle_travelled = atan(robot_y_distance / robot_x_distance);
	field_angle_travelled = gyro->getAngle() - robot_angle_travelled;
}

void FieldMapping::GetCoordinates(positions p, float* output)
{
	int pos = p;
	switch(pos) {
	case STAGING_1:
		output[0] = 1.0;
		output[1] = 1.0;
		break;
	case STAGING_2:
		output[0] = 2.0;
		output[1] = 2.0;
		break;
	case STAGING_3:
		output[0] = 3.0;
		output[1] = 3.0;
		break;
	}
}

FieldMapping* FieldMapping::getInstance()
{
	if (INSTANCE == NULL) {
		INSTANCE = new FieldMapping();
	}
	return INSTANCE;
}

