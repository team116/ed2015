#include <ctime>
namespace RobotPorts
{

	// motors
	const unsigned int FRONT_LEFT_MOTOR = 1;
	const unsigned int FRONT_RIGHT_MOTOR = 2;
	const unsigned int REAR_LEFT_MOTOR = 3;
	const unsigned int REAR_RIGHT_MOTOR = 4;
	const unsigned int LEFT_WHEEL = 11;
	const unsigned int RIGHT_WHEEL = 5;
	const unsigned int LIFTER_ONE = 6;
	const unsigned int LIFTER_TWO = 7;
	const unsigned int RAKE_PORT_MOTOR = 8;
	const unsigned int RAKE_STARBOARD_MOTOR = 9;
	const unsigned int CLOSE_FLAPS_MOTOR = 10;

	// analog sensors
	const unsigned int GYRO = 0;
	const unsigned int AUTONOMOUS_DELAY_SWITCH = 1;
	const unsigned int AUTONOMOUS_PLAY_SWITCH = 2;
	const unsigned int AUTONOMOUS_LOCATION_SWITCH = 3;
	const unsigned int ULTRASONIC = 4;
	const unsigned int FLAP_POTENTIOMETER = 5;

	// digital sensors
	//limits run through motors, not RoboRio
	/*const unsigned int LIFT_UPPER_LIMIT = 0;
	const unsigned int LIFT_LOWER_LIMIT = 1;
	const unsigned int FLAPS_UPPER_LIMIT = 2;
	const unsigned int FLAPS_LOWER_LIMIT = 3;
	const unsigned int PORT_RAKE_LIMIT = 4;
	const unsigned int STARBOARD_RAKE_LIMIT = 5;*/

	const unsigned int ENCODER_A = 6;
	const unsigned int ENCODER_B = 7;
	const unsigned int ODOMETRY_WHEEL_X_A = 0;
	const unsigned int ODOMETRY_WHEEL_X_B = 1;
	const unsigned int ODOMETRY_WHEEL_Y_A = 2;
	const unsigned int ODOMETRY_WHEEL_Y_B = 3;

	//PWM
	const unsigned int LEFT_TREX_ARM = 0;
	const unsigned int RIGHT_TREX_ARM = 1;
	const unsigned int LEFT_RAKE_STABILIZER = 2;
	const unsigned int RIGHT_RAKE_STABILIZER = 3;
	//const unsigned int
}

namespace DSPorts
{
	//joysticks
	const unsigned int DRIVER_ONE_JOYSTICK = 0;
	const unsigned int DRIVER_TWO_JOYSTICK = 1;
	const unsigned int OUTPUT_BOARD = 2;
	const unsigned int INPUT_BOARD = 3;

}

namespace JoystickPorts
{
	const unsigned int TOGGLE_ROTATION = 2;
	const unsigned int FIELD_CENTRIC_TOGGLE = 3;
	const unsigned int OVERRIDE_BUTTON = 7;
	const unsigned int TURN_DEGREES = 5;
	const unsigned int FLIP_ORIENTATION = 11;
	const unsigned int TEMP_CAMERA_TOGGLE_TEST = 1; //to be removed, used only to test camera toggleS

}

namespace LEDBoardPorts
{
	//digital outputs
	const unsigned int LEVEL_0_INDICATOR = 12;
	const unsigned int LEVEL_1_INDICATOR = 13;
	const unsigned int LEVEL_2_INDICATOR = 14;
	const unsigned int LEVEL_3_INDICATOR = 15;
	const unsigned int LEVEL_4_INDICATOR = 16;
	const unsigned int LEVEL_5_INDICATOR = 17;
	const unsigned int LEVEL_6_INDICATOR = 18;

	/*
	const unsigned int STACK_ON_FLOOR_INDICATOR = 19;
	const unsigned int STACK_ON_PLATFORM_INDICATOR = 20;
	const unsigned int STACK_ON_STEP_INDICATOR = 21;

	const unsigned int BACK_CAMERA_INDICATOR = 22;
	const unsigned int FRONT_CAMERA_INDICATOR = 23;
	*/
}

namespace InputBoardPorts
{
	//digital inputs
	const unsigned int LIFTER_PRESET_0 = 1;
	const unsigned int LIFTER_PRESET_1 = 2;
	const unsigned int LIFTER_PRESET_2 = 3;
	const unsigned int LIFTER_PRESET_3 = 4;
	const unsigned int LIFTER_PRESET_4 = 5;
	const unsigned int LIFTER_PRESET_5 = 6;
	const unsigned int LIFTER_PRESET_6 = 7;

	const unsigned int LIFTER_DOWN_BUTTON = 8;
	const unsigned int LIFTER_UP_BUTTON = 9;

	//These currently don't appear to be wired up. OI NEEDS to be contacted to figure out why
	const unsigned int STACK_ON_PLATFORM_SWITCH = 99;
	const unsigned int STACK_ON_STEP_SWITCH = 100;

	const unsigned int CAMERA_SELECT_SWITCH = 12;

	const unsigned int LEFT_RAKE_UP_BUTTON = 14;
	const unsigned int LEFT_RAKE_DOWN_BUTTON = 13;

	const unsigned int RIGHT_RAKE_UP_BUTTON = 16;
	const unsigned int RIGHT_RAKE_DOWN_BUTTON = 15;

	//analog inputs
	const unsigned int FLAP_POSITION_KNOB = 1;
}
/*
namespace IOBoardOnePorts
{
	//digital inputs
	const unsigned int LIFTER_PRESET_0 = 1;
	const unsigned int LIFTER_PRESET_1 = 2;
	const unsigned int LIFTER_PRESET_2 = 3;
	const unsigned int LIFTER_PRESET_3 = 4;
	const unsigned int LIFTER_PRESET_4 = 5;
	const unsigned int LIFTER_PRESET_5 = 6;
	const unsigned int LIFTER_PRESET_6 = 7;

	const unsigned int STACK_ON_PLATFORM_SWITCH = 8;
	const unsigned int STACK_ON_STEP_SWITCH = 9;

	const unsigned int LIFTER_UP_BUTTON = 10;
	const unsigned int LIFTER_DOWN_BUTTON = 11;

	//digital outputs
	const unsigned int LEVEL_0_INDICATOR = 12;
	const unsigned int LEVEL_1_INDICATOR = 13;
	const unsigned int LEVEL_2_INDICATOR = 14;
	const unsigned int LEVEL_3_INDICATOR = 15;
	const unsigned int LEVEL_4_INDICATOR = 16;
	const unsigned int LEVEL_5_INDICATOR = 17;
	const unsigned int LEVEL_6_INDICATOR = 18;

	const unsigned int STACK_ON_FLOOR_INDICATOR = 19;
	const unsigned int STACK_ON_PLATFORM_INDICATOR = 20;
	const unsigned int STACK_ON_STEP_INDICATOR = 21;

	const unsigned int BACK_CAMERA_INDICATOR = 22;
	const unsigned int FRONT_CAMERA_INDICATOR = 23;

	//analog inputs
	const unsigned int FLAP_POSITION_KNOB = 1;
}

namespace IOBoardTwoPorts{
	const unsigned int FRONT_CAMERA_SELECT = 1;
	const unsigned int BACK_CAMERA_SELECT = 2;

	const unsigned int LEFT_RAKE_UP_BUTTON = 3;
	const unsigned int LEFT_RAKE_DOWN_BUTTON = 4;
	const unsigned int RIGHT_RAKE_UP_BUTTON = 5;
	const unsigned int RIGHT_RAKE_DOWN_BUTTON = 6;
}
*/
namespace Utils
{
	int convertFromVolts(const float voltage, const int voltage_levels, const float max_voltage);
	char* getCurrentTime();
}
