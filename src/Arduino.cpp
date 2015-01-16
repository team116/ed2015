#include "Log.h"
#include "Arduino.h"
#include "Ports.h"
#include <WPILib.h>
#include <gyro.h>
#include "Mobility.h"
#include "DS.h"


Arduino::Arduino() {
	log = Log::getInstance();
	mobility = Mobility::getInstance();
	driver_station = DS::getInstance();
	//some_limit = new DigitalInput(RobotPorts::SOME_LIMIT);

	myAccel = new ADXL345_I2C(1, ADXL345_I2C::kRange_2G);
	//ultra_sonic = new AnalogChannel(RobotPorts::FRONT_ULTRASONIC_SENSOR);	//figure out where AnalogChannel is supposed to come from
	//gyro = new Gyro(1,1);

}

Arduino::~Arduino() {
	// TODO Auto-generated destructor stub
}

void Arduino::process() {

}

