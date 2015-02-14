/*
 * Sensors.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Lenovo
 */

#include <Encoder.h>
#include "Sensors.h"
#include "Log.h"

Sensors* Sensors::INSTANCE = NULL;

Sensors::Sensors()
{
	od_xaxis_encoder = new Encoder(RobotPorts::OD_XAXIS_ENCODER_A, RobotPorts::OD_XAXIS_ENCODER_B);
	od_yaxis_encoder = new Encoder(RobotPorts::OD_YAXIS_ENCODER_A, RobotPorts::OD_YAXIS_ENCODER_B);
}

void Sensors::process()
{
	log->write(Log::INFO_LEVEL, "Get: Front Left Encoder %i\n", od_xaxis_encoder->Get());
	log->write(Log::INFO_LEVEL, "Get: Back Right Encoder %i\n", od_yaxis_encoder->Get());
}

Sensors* Sensors::getInstance()
{
	if (INSTANCE == NULL) {
		INSTANCE = new Sensors();
	}
	return INSTANCE;
}
