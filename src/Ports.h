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

	// digital sensors
	const unsigned int LIFT_UPPER_LIMIT = 0;
	const unsigned int LIFT_LOWER_LIMIT = 1;
	const unsigned int FLAPS_UPPER_LIMIT = 2;
	const unsigned int FLAPS_LOWER_LIMIT = 3;
	const unsigned int PORT_RAKE_LIMIT = 4;
	const unsigned int STARBOARD_RAKE_LIMIT = 5;

	const unsigned int ENCODER_A = 6;
	const unsigned int ENCODER_B = 7;

}

namespace DSPorts
{
	//joysticks
	const unsigned int DRIVER_ONE_JOYSTICK = 0;
	const unsigned int DRIVER_TWO_JOYSTICK = 1;
	const unsigned int DIGITAL_IO_BOARD = 2;
	const unsigned int SECOND_IO_BOARD = 3;

}

namespace JoystickPorts
{
	const unsigned int FIELD_CENTRIC_TOGGLE = 2;
	const unsigned int OVERRIDE_BUTTON = 7;
}

namespace IOBoardOnePorts
{
	//digital inputs
	const unsigned int LIFTER_PRESET_1 = 1;
	const unsigned int LIFTER_PRESET_2 = 2;
	const unsigned int LIFTER_PRESET_3 = 3;
	const unsigned int LIFTER_PRESET_4 = 4;
	const unsigned int LIFTER_PRESET_5 = 5;
	const unsigned int LIFTER_PRESET_6 = 6;

	const unsigned int STACK_ON_PLATFORM_SWITCH = 7;
	const unsigned int STACK_ON_STEP_SWITCH = 8;

	const unsigned int LIFTER_UP_BUTTON = 9;
	const unsigned int LIFTER_DOWN_BUTTON = 10;

	//digital outputs
	const unsigned int LEVEL_0_INDICATOR = 13;
	const unsigned int LEVEL_1_INDICATOR = 14;
	const unsigned int LEVEL_2_INDICATOR = 15;
	const unsigned int LEVEL_3_INDICATOR = 16;
	const unsigned int LEVEL_4_INDICATOR = 17;
	const unsigned int LEVEL_5_INDICATOR = 18;

	const unsigned int STACK_ON_FLOOR_INDICATOR = 19;
	const unsigned int STACK_ON_PLATFORM_INDICATOR = 20;
	const unsigned int STACK_ON_STEP_INDICATOR = 21;

	const unsigned int BACK_CAMERA_INDICATOR = 22;
	const unsigned int FRONT_CAMERA_INDICATOR = 23;

	//analog inputs
	const unsigned int ROTATION_KNOB = 2;
}

namespace IOBoardTwoPorts{
	const unsigned int FRONT_CAMERA_SELECT = 1;
	const unsigned int BACK_CAMERA_SELECT = 2;

	const unsigned int RAKES_UP_BUTTON = 3;
	const unsigned int RAKES_DOWN_BUTTON = 4;
}

namespace Utils
{
	int convertFromVolts(const float voltage, const int voltage_levels, const float max_voltage);
	char* getCurrentTime();
}
