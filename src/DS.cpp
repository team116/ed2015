#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
using namespace std;
DS* DS::INSTANCE = NULL;

DS::DS()
{
	test_gamepad = Joystick::GetStickForPort(DSPorts::TEST_GAMEPAD);
	mobility = Mobility::getInstance();
}

void DS::process()
{
	mobility->setDirection(test_gamepad->GetRawAxis(4), test_gamepad->GetRawAxis(5));
	mobility->setRotation(test_gamepad->GetRawAxis(0));
}

DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
