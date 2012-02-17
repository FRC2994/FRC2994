#include "MBUltrasonic.h"
#include "WPILib.h"


/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
//	RobotDrive myRobot; // robot drive system
//	Joystick stick; // only joystick

public:
	Joystick *left;
	DriverStationLCD *ds_lcd;
	RobotDemo(void)
	{
		left = new Joystick(2);
		ds_lcd = DriverStationLCD::GetInstance();
		ds_lcd->Printf(DriverStationLCD::kUser_Line1, 1, "UT z20120216 1520");
		ds_lcd->UpdateLCD();
		// myRobot.SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		MBUltrasonic *sonic = new MBUltrasonic(2, 3);
		sonic->PingEnable(true);
		double range = 0;
		// myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			range = sonic->GetRangeInches();
			if (range >= 0)
			{
				ds_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "%f", range);
				ds_lcd->UpdateLCD();
			}
			Wait(0.05);
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

