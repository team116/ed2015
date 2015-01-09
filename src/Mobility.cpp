#include "WPILib.h"
#include "Ports.h"
#include "Mobility.h"
#include "Gyro.h"

Mobility* Mobility::INSTANCE = NULL;

Mobility::Mobility()
{

}

void Mobility::process()
{

}

void Mobility::setSpeed(float speed)
{

}

void Mobility::setDirection(int speed)
{

}

Mobility* Mobility::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Mobility();
    }
    return INSTANCE;
}
