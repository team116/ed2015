#include <WPILib.h>
#include "Ports.h"
#include "Log.h"
#include "DS.h"

class Robot : public IterativeRobot
{
private:
	DS* ds;
	Log* log;

public:
	Robot(void)
	{
		ds = DS::getInstance();
		log = Log::getInstance();
	}

    ////////////////////////////////////////////////////////////////////////////
    //****************************** Init Routines *****************************
    ////////////////////////////////////////////////////////////////////////////

    void RobotInit()
    {

    }

    void DisabledInit()
    {
    	// write log functions
    }

    void AutonomousInit()
    {

    }

    void TeleopInit()
    {

    }

    ////////////////////////////////////////////////////////////////////////////
    //**************************** Periodic Routines ***************************
    ////////////////////////////////////////////////////////////////////////////

    void DisabledPeriodic()
    {

    }

    void AutonomousPeriodic()
    {

    }

    void TeleopPeriodic()
    {
    	ds->process();
    }

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
