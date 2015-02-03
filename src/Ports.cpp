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
time_t now;
int convertFromVolts(const float voltage, const int voltage_levels,
		const float max_voltage) {
	float output = (voltage / max_voltage) * ((float) voltage_levels - 1); // needs to be rounded, never do == w/ decimals
	return (int) (output + 0.5); // round the value
}
char* getCurrentTime() {
	struct tm* time_struct;
	time_t t;
	time(&t);
	time_struct = localtime(&t);

	int hour = time_struct->tm_hour;
	int min = time_struct->tm_min;
	int sec = time_struct->tm_sec;
	char* temp = "";
	sprintf(temp,"%i:%i:%i", hour, min, sec);
	return temp;
}
}
