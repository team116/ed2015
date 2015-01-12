#include "WPILib.h"
#include "Ports.h"
#include "Log.h"
#include "DS.h"
#include "Mobility.h"
#include "Autonomous.h"
#include "Manipulator.h"

class Robot : public IterativeRobot
{
private:
	// subsytem instance pointers
	Mobility* mobility;
	DS* ds;
	Autonomous* autonomous;
	Manipulator* manipulator;
	Log* log;

	LiveWindow* lw;

public:
	Robot(void)
	{
		// initialize subsystems
		ds = DS::getInstance();
		mobility = Mobility::getInstance();
		autonomous = Autonomous::getInstance();
		manipulator = Manipulator::getInstance();
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
    	autonomous->process();
    }

    void TeleopPeriodic()
    {
    	ds->process();
    	manipulator->process();
    }

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
