#include "WPILib.h"
#include "Ports.h"
#include "Log.h"
#include "DS.h"
#include "Mobility.h"
#include "Autonomous.h"
#include "Manipulator.h"
#include "I2CCompass.h"

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
	I2CCompass* compass;
	//I2CGyro* gyro;

public:
	Robot(void)
	{
		// initialize subsystems
		log = Log::getInstance();
		ds = DS::getInstance();
		mobility = Mobility::getInstance();
		manipulator = Manipulator::getInstance();
		compass = I2CCompass::getInstance();
		//gyro = I2CGyro::getInstance();
		// initialize autonomous switches
		delay_switch = new AnalogInput(RobotPorts::AUTONOMOUS_DELAY_SWITCH);
		play_switch = new AnalogInput(RobotPorts::AUTONOMOUS_PLAY_SWITCH);
		location_switch = new AnalogInput(RobotPorts::AUTONOMOUS_LOCATION_SWITCH);
		log->write(Log::ERROR_LEVEL, "%s\tRobot construction finished\n", Utils::getCurrentTime());
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
    	log->write(Log::ERROR_LEVEL, "%s\tAutonomous Init called\n", Utils::getCurrentTime());
    	//Dont set control mode here, set it for individual plays
    	// mobility->setControlMode(CANTalon::kPosition);
		const float max_volt = 5.2;
		float delay_volt = delay_switch->GetVoltage();
		float play_volt = play_switch->GetVoltage();
		float location_volt = location_switch->GetVoltage();
		log->write(Log::ERROR_LEVEL,"Play Voltage: %f\nDelay Voltage: %f\n Location Voltage: %f\n", play_volt, delay_volt, location_volt);

		int delay_pos = Utils::convertFromVolts(delay_volt, 6, max_volt);
		int play_pos = Utils::convertFromVolts(play_volt, 6, max_volt);
		int loc_pos = Utils::convertFromVolts(location_volt, 6, max_volt);
		autonomous = new Autonomous(delay_pos, play_pos, loc_pos);
    	log->write(Log::ERROR_LEVEL, "%s\tAutonomous Init ended\n", Utils::getCurrentTime());
    }

    void TeleopInit()
    {
		log->write(Log::ERROR_LEVEL, "%s\tTeleop init started\n", Utils::getCurrentTime());
    	mobility->setControlMode(CANTalon::kSpeed);
		log->write(Log::ERROR_LEVEL, "%s\tTeleop init finished\n", Utils::getCurrentTime());
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
    	compass->process();
    	//gyro->process();
    }

    void TeleopPeriodic()
    {
    	ds->process();
    	mobility->process();
    	manipulator->process();
    	compass->process();
    	//gyro->process();
    }

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot);
