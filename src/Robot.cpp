#include <WPILib.h>
#include <AnalogInput.h>
#include "Ports.h"
#include "Log.h"
#include "DS.h"
#include "Motors.h"

class Robot : public IterativeRobot
{
private:
	DS* ds;
	Motors* motors;
	Log* log;
	AnalogInput* autonomous_delay_switch;
	AnalogInput* autonomous_play_switch;
	AnalogInput* autonomous_location_switch;
public:
	Robot(void)
	{
		ds = DS::getInstance();
		log = Log::getInstance();
		motors = Motors::getInstance();
		autonomous_delay_switch = new AnalogInput(RobotPorts::AUTONOMOUS_DELAY_SWITCH);
		autonomous_location_switch = new AnalogInput(RobotPorts::AUTONOMOUS_LOCATION_SWITCH);
		autonomous_play_switch = new AnalogInput(RobotPorts::AUTONOMOUS_PLAY_SWITCH);
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
    	log->write(Log::INFO_LEVEL,"Delay switch %i\n", Utils::convertFromVolts(autonomous_delay_switch->GetVoltage(), 6, 5.0));
    	log->write(Log::INFO_LEVEL,"Play switch  %i\n", Utils::convertFromVolts(autonomous_play_switch->GetVoltage(), 6, 5.0));
    	log->write(Log::INFO_LEVEL,"Location switch %i\n", Utils::convertFromVolts(autonomous_location_switch->GetVoltage(), 6, 5.0));
    }

    void TeleopInit()
    {
    	log->write(Log::INFO_LEVEL,"Delay switch %i\n", Utils::convertFromVolts(autonomous_delay_switch->GetVoltage(), 6, 5.0));
    	log->write(Log::INFO_LEVEL,"Play switch  %i\n", Utils::convertFromVolts(autonomous_play_switch->GetVoltage(), 6, 5.0));
    	log->write(Log::INFO_LEVEL,"Location switch %i\n", Utils::convertFromVolts(autonomous_location_switch->GetVoltage(), 6, 5.0));
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
    	motors->process();
    	ds->process();
    }

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
