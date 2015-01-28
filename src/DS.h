#ifndef DS_H_
#define DS_H_
#include <CANTalon.h>

class DS
{
public:
	static DS* getInstance();

	void process();

private:
	DS();
	static DS* INSTANCE;
CANTalon* front_left_wheel;
CANTalon* front_right_wheel;
CANTalon* rear_left_wheel;
CANTalon* rear_right_wheel;
CANTalon* left_grabber_wheel;
CANTalon* right_grabber_wheel;
CANTalon* lifter_one;
CANTalon* lifter_two;
CANTalon* rake_port;
CANTalon* rake_starboard;
CANTalon* flaps;
Joystick* joystick_one;
Joystick* joystick_two;
Joystick* joystick_three;
};

#endif // DS_H_
