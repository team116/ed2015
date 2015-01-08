#include "WPILib.h"
#include "Ports.h"
#include "Mobility.h"

Mobility* Mobility::INSTANCE = NULL;

Mobility::Mobility()
{

}

Mobility* Mobility::getInstance()
{
    if (INSTANCE == NULL) {
        INSTANCE = new Mobility();
    }
    return INSTANCE;
}