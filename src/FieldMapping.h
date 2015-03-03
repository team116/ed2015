/*
 * FieldMapping.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Will
 */

#ifndef SRC_FIELDMAPPING_H_
#define SRC_FIELDMAPPING_H_

#include "Mobility.h"
#include <BuiltInAccelerometer.h>

class FieldMapping {
public:
	static FieldMapping* getInstance();

	void process();
private:
	static FieldMapping* INSTANCE;
	FieldMapping();

	static const float ROBOT_WIDTH;
	static const float ROBOT_LENGTH;

	BuiltInAccelerometer* accel;
	I2CGyro* gyro;

	float x_pos;
	float y_pos;
	float excuse_me_question_mark [2];

	float angle;
	float x_velocity;
	float y_velocity;

	Timer* velocity_timer;

	int starting_pos;

	enum positions
	{
		STAGING_1,
		STAGING_2,
		STAGING_3
	};

	void GetCoordinates(positions p, float* output);
};

#endif /* SRC_FIELDMAPPING_H_ */
