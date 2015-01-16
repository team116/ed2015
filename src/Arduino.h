#ifndef SRC_ARDUINO_H_
#define SRC_ARDUINO_H_

#include "Log.h"
#include "WPILib.h"
#include "Ports.h"
#include <gyro.h>
#include "Mobility.h"
#include "DS.h"

class Arduino {
public:

	  static Arduino* get_instance();
	  //bool get_some_limit(); however many times as needed

	  void process();

	  float x_accelerometer();
	  float y_accelerometer();
	  float z_accelerometer();
	  float get_distance();

	  void display_sensors();

	  DS* driver_station;

  private:

	  Log* log;
	  Mobility* mobility;
	  //DigitalInput* some_limit;
	  ADXL345_I2C* myAccel;
	  //AnalogChannel* ultra_sonic;
//	  Gyro* gyro;

	  /***figure out what all this does, adapt it***/
	  /*int ball;
	  int cock;
	  int intake;
	  int gear_top;
	  int gear_bot;
	  int intake_piston_in;
	  int intake_piston_out;
	  int catcher_in;
	  int catcher_out;
	  int pressure;

	  bool ball_true;
	  bool cock_true;
	  bool intake_true;
	  bool gear_top_true;
	  bool gear_bot_true;
	  bool intake_piston_in_true;
	  bool intake_piston_out_true;
	  bool catcher_in_true;
	  bool catcher_out_true;
	  bool pressure_true;*/

	  static Arduino* INSTANCE;

	  Arduino();
 	 ~Arduino();
};

#endif /* SRC_ARDUINO_H_ */
