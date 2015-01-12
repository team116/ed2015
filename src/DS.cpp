#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
using namespace std;
DS* DS::INSTANCE = NULL;

DS::DS()
{

}

void DS::process()
{

}

DS* DS::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new DS();
	}
	return INSTANCE;
}
