#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
#include "Autonomous.h"

class Robot : public IterativeRobot
{
private:
	// subsytem instance pointers
	Mobility* mobility;
	DS* ds;
	Autonomous* autonomous;

	LiveWindow* lw;

public:
	Robot(void)
	{
		// initialize subsystems
		ds = DS::getInstance();
		mobility = Mobility::getInstance();
		autonomous = Autonomous::getInstance();
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

    }

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);