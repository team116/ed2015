namespace RobotPorts
{

	// motors
	const unsigned int FRONT_LEFT_MOTOR = 1;
	const unsigned int FRONT_RIGHT_MOTOR = 2;
	const unsigned int REAR_LEFT_MOTOR = 3;
	const unsigned int REAR_RIGHT_MOTOR = 4;
	const unsigned int LEFT_WHEEL = 5;
	const unsigned int RIGHT_WHEEL = 6;
	const unsigned int LIFTER_ONE = 7;
	const unsigned int LIFTER_TWO = 8;
	const unsigned int RAKE_PORT_MOTOR = 9;
	const unsigned int RAKE_STARBOARD_MOTOR = 10;
	const unsigned int CLOSE_HOOKS_MOTOR = 11;

	// analog sensors
	const unsigned int GYRO = 0;
	const unsigned int ULTRASONIC = 1;

	// digital sensors


}

namespace DSPorts
{
//joysticks
const unsigned int DRIVER_ONE_JOYSTICK = 0;
const unsigned int DRIVER_TWO_JOYSTICK = 2;
const unsigned int BUTTONS_JOYSTICK = 3;

//analog inputs
const unsigned int CONTAINER_RAKE_HEIGHT = 1;

//umm idk about leds
const unsigned int LIFTER_LED_1 = 1;
const unsigned int LIFTER_LED_2 = 2;
const unsigned int LIFTER_LED_3 = 3;
const unsigned int LIFTER_LED_4 = 4;
const unsigned int LIFTER_LED_5 = 5;
const unsigned int LIFTER_LED_6 = 6;
const unsigned int ON_STEP_LED = 7;
const unsigned int OFF_STEP_LED = 8;

}

namespace JoystickPorts
{
const unsigned int OVERRIDE_BUTTON = 1;
}

namespace ButtonPorts
{
//I'm putting these in their own namespace and making them pretend that they're a joystick
//because apparently DigitalInput isn't actually meant to be used and I know how to do this
const unsigned int LIFTER_PRESET_1 = 1;
const unsigned int LIFTER_PRESET_2 = 12;
const unsigned int LIFTER_PRESET_3 = 3;
const unsigned int LIFTER_PRESET_4 = 4;
const unsigned int LIFTER_PRESET_5 = 11;
const unsigned int LIFTER_PRESET_6 = 6;
const unsigned int STACK_ON_PLATFORM_SWITCH = 7;
const unsigned int STACK_ON_STEP_SWITCH = 8;
const unsigned int MOVE_UP_BUTTON = 9;
const unsigned int MOVE_DOWN_BUTTON = 10;
const unsigned int DRIVE_TYPE = 2;

}
