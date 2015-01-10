#include "WPILib.h"
#include "Ports.h"
#include "Autonomous.h"

Autonomous* Autonomous::INSTANCE = NULL;

Autonomous::Autonomous()
{

}

void Autonomous::process()
{

}

Autonomous* Autonomous::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Autonomous();
    }
    return INSTANCE;
}
