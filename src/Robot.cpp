#include "WPILib.h"
#include "Ports.h"
#include "Log.h"
#include "DS.h"
#include "Mobility.h"
#include "Autonomous.h"
#include "Manipulator.h"
#include "SensorStick.h"

class Robot : public IterativeRobot
{
private:
	// autonomous switches
	AnalogInput* delay_switch;
	AnalogInput* play_switch;
	AnalogInput* location_switch;

	// subsytem instance pointers
	Mobility* mobility;
	DS* ds;
	Autonomous* autonomous;
	Manipulator* manipulator;
	Log* log;
	SensorStick* sensorstick;

public:
	Robot(void)
	{
		// initialize autonomous switches
		delay_switch = new AnalogInput(RobotPorts::AUTONOMOUS_DELAY_SWITCH);
		play_switch = new AnalogInput(RobotPorts::AUTONOMOUS_PLAY_SWITCH);
		location_switch = new AnalogInput(RobotPorts::AUTONOMOUS_LOCATION_SWITCH);
		float delay_volt = delay_switch->GetVoltage();
		float play_volt = play_switch->GetVoltage();
		float location_volt = location_switch->GetVoltage();
		const float max_volt = 5.2;

		// initialize subsystems
		ds = DS::getInstance();
		mobility = Mobility::getInstance();
		autonomous = Autonomous::getInstance(Utils::convertFromVolts(delay_volt, 0, max_volt), Utils::convertFromVolts(play_volt, 0, max_volt), Utils::convertFromVolts(location_volt, 0, max_volt));
		manipulator = Manipulator::getInstance();
		log = Log::getInstance();
		sensorstick = SensorStick::getInstance();
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
    	mobility->process();
    	manipulator->process();
    	sensorstick ->process();
    }

    void TeleopPeriodic()
    {
    	ds->process();
    	mobility->process();
    	manipulator->process();
    	sensorstick ->process();

    	log ->write(Log::INFO_LEVEL, "AccelX: %i\n");
    	log ->write(Log::INFO_LEVEL, "AccelY: %i\n");
		log ->write(Log::INFO_LEVEL, "AccelX: %i\n");
    	log -> write(Log::INFO_LEVEL, "Compass Yaw: %f\n");
    }

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
