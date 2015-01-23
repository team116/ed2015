/*
 * Ports.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: Lenovo
 */


namespace Utils {
	int convertFromVolts(const float voltage, const int voltage_levels, const float max_voltage) {
		float output = (voltage / max_voltage) * ((float)voltage_levels-1); // needs to be rounded, never do == w/ decimals
		return (int)(output + 0.5); // round the value
	}
}
