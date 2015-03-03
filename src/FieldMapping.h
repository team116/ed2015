/*
 * FieldMapping.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Will
 */

#ifndef SRC_FIELDMAPPING_H_
#define SRC_FIELDMAPPING_H_

#include "Mobility.h"

class FieldMapping {
public:
	static FieldMapping* getInstance();

	void process();
private:
	static FieldMapping* INSTANCE;
	FieldMapping();

	static const float ROBOT_WIDTH;
	static const float ROBOT_LENGTH;

	float x_pos;
	float y_pos;

	int starting_pos;

	enum positions
	{
		STAGING_1,
		STAGING_2,
		STAGING_3
	};
};

#endif /* SRC_FIELDMAPPING_H_ */
