#include "WPILib.h"
#include "Ports.h"
#include "DS.h"

DS* DS::INSTANCE = NULL;

DS::DS()
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