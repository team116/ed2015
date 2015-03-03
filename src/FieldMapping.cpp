/*
 * FieldMapping.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Will
 */

#include "FieldMapping.h"
#include "Mobility.h"

FieldMapping* FieldMapping::INSTANCE = NULL;

//TODO:Get actual robot dimensions
const float FieldMapping::ROBOT_WIDTH = 0.0;
const float FieldMapping::ROBOT_LENGTH = 0.0;

FieldMapping::FieldMapping()
{
	//in inches
	x_pos = 0,0;
	y_pos = 0.0;

	starting_pos = STAGING_1;
	switch(starting_pos) {
	case STAGING_1:
		x_pos = 0.0;
		y_pos = 35 - (ROBOT_LENGTH / 2);
		break;
	case STAGING_2:
		x_pos = 0.0;
		y_pos = 35 - (ROBOT_LENGTH / 2);
		break;
	case STAGING_3:
		x_pos = 0.0;
		y_pos = 35 - (ROBOT_LENGTH / 2);
		break;
	}

}

void FieldMapping::process()
{

}

FieldMapping* FieldMapping::getInstance()
{
	if (INSTANCE == NULL) {
		INSTANCE = new FieldMapping();
	}
	return INSTANCE;
}

