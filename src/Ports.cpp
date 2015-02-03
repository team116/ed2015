/*
 * Ports.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Lenovo
 */
#include "Ports.h"
#include <ctime>

namespace Utils {
time_t now;
int convertFromVolts(const float voltage, const int voltage_levels,
		const float max_voltage) {
	float output = (voltage / max_voltage) * ((float) voltage_levels - 1); // needs to be rounded, never do == w/ decimals
	return (int) (output + 0.5); // round the value
}
char* getCurrentTime() {
	struct tm* time;
	time_t t;
	time(&t);
	time = localtime(&t);

	int hour = time->tm_hour;
	int min = time->tm_min;
	int sec = time->tm_sec;
	return "%i:%i:%i", hour, min, sec;
}
}
