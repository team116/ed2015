#include <ctime>

namespace RobotPorts
{

	// motors
	const unsigned int FRONT_LEFT_MOTOR = 1;
	const unsigned int FRONT_RIGHT_MOTOR = 2;
	const unsigned int REAR_LEFT_MOTOR = 3; //3
	const unsigned int REAR_RIGHT_MOTOR = 4;
	const unsigned int LEFT_LIFTER_WHEEL = 11;
	const unsigned int RIGHT_LIFTER_WHEEL = 5;
	const unsigned int LIFTER_ONE = 6;
	//const unsigned int LIFTER_TWO = 7;
	const unsigned int RAKE_PORT_MOTOR = 7; //8
	const unsigned int RAKE_STARBOARD_MOTOR = 12; //9
	const unsigned int CLOSE_FLAPS_MOTOR = 10;
	const unsigned int LEFT_BASE_WHEEL = 8;
	const unsigned int RIGHT_BASE_WHEEL = 9;

	// analog sensors
	const unsigned int AUTONOMOUS_DELAY_SWITCH = 1;
	const unsigned int AUTONOMOUS_PLAY_SWITCH = 2;
	const unsigned int AUTONOMOUS_LOCATION_SWITCH = 3;
//	const unsigned int ULTRASONIC = 0;
	const unsigned int GYRO = 0;

	// digital sensors
	//limits run through talons, not RoboRio

	const unsigned int ENCODER_A = 6;
	const unsigned int ENCODER_B = 7;
	const unsigned int ODOMETRY_WHEEL_X_A = 0;
	const unsigned int ODOMETRY_WHEEL_X_B = 1;
	const unsigned int ODOMETRY_WHEEL_Y_A = 2;
	const unsigned int ODOMETRY_WHEEL_Y_B = 3;

	// PWM
	const unsigned int LEFT_TREX_ARM = 0;
	const unsigned int RIGHT_TREX_ARM = 4;
	const unsigned int LEFT_RAKE_STABILIZER = 1;
	const unsigned int RIGHT_RAKE_STABILIZER = 3;
	const unsigned int CONTAINER_BLOCKER = 2;
	//const unsigned int

	// I2C, everything in hex cuz why the hell not
	const unsigned int GYRO_ADDRESS = 0x68;
	const unsigned int GYRO_REG_POWER = 0x3E;
	const unsigned int GYRO_REG_DLPF_FS = 0x16;
	const unsigned int GYRO_REG_SAMPLE_RATIO = 0x15;
	const unsigned int GYRO_REG_PLL = 0x3E;
	const unsigned int GYRO_REG_MXSB = 0xD1;

	const unsigned int COMPASS_ADDRESS = 0x1E;
	const unsigned int COMPASS_REG_MEASURE_MODE = 0x02;
	const unsigned int COMPASS_REG_XMSB = 0x03;

	const unsigned int ACCEL_ADDRESS = 0x53;
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
	const unsigned int USE_BASE_WHEELS = 1;
	const unsigned int CARDINAL_DIRECTION = 1;
	const unsigned int TOGGLE_ROTATION = 2;
	const unsigned int FIELD_CENTRIC_TOGGLE = 3;
	const unsigned int OVERRIDE_BUTTON = 7;
	const unsigned int TURN_DEGREES = 5;
	const unsigned int FLIP_ORIENTATION = 11;
	const unsigned int TEMP_CAMERA_TOGGLE_TEST = 4; //to be removed, used only to test camera toggleS

}

namespace LEDBoardPorts
{
	//digital outputs
	const unsigned int LEVEL_0_INDICATOR = 7;
	const unsigned int LEVEL_1_INDICATOR = 6;
	const unsigned int LEVEL_2_INDICATOR = 5;
	const unsigned int LEVEL_3_INDICATOR = 4;
	const unsigned int LEVEL_4_INDICATOR = 3;
	const unsigned int LEVEL_5_INDICATOR = 2;
	const unsigned int LEVEL_6_INDICATOR = 1;
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

	const unsigned int STACK_ON_FLOOR_SWITCH = 17;
	const unsigned int STACK_ON_STEP_SWITCH = 18;

	const unsigned int CAMERA_SELECT_SWITCH = 12;

	const unsigned int LEFT_RAKE_DOWN_BUTTON = 13;
	const unsigned int LEFT_RAKE_UP_BUTTON = 14;

	const unsigned int RIGHT_RAKE_DOWN_BUTTON = 15;
	const unsigned int RIGHT_RAKE_UP_BUTTON = 16;

	const unsigned int FLAP_LOWER = 10;
	const unsigned int FLAP_RAISE = 11;


	//analog inputs
	const unsigned int FLAP_POSITION_KNOB = 0;
}

namespace Utils
{
	int convertFromVolts(const float voltage, const int voltage_levels, const float max_voltage);
	char* getCurrentTime();
}
