/*
 * Ports.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Lenovo
 */
#include "Ports.h"
#include <ctime>
#include <cstdio>

namespace Utils {
	int convertFromVolts(const float voltage, const int voltage_levels,
			const float max_voltage)
	{
		float output = (voltage / max_voltage) * ((float) voltage_levels - 1); // needs to be rounded, never do == w/ decimals
		return (int) (output + 0.5); // round the value
	}

	char latest_time_stamp[9]; // avoids dynamic memory allocation
	char* getCurrentTime()
	{
		struct tm* local_time;
		time_t t;
		time(&t);
		local_time = localtime(&t);

		sprintf(latest_time_stamp, "%02i:%02i:%02i", local_time->tm_hour, local_time->tm_min, local_time->tm_sec);
		return latest_time_stamp;
	}
}
