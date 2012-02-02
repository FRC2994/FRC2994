#include "WPILib.h"
#include "DashboardDataSender.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	Compressor *compressor;
	Joystick *stick; // only joystick
	Jaguar *motor;
	Solenoid *valve;
	DashboardDataSender *	dds;	// to enable real-time update of hw state to the dashboard
	DriverStationLCD *dsLCD;


public:
	RobotDemo(void)
	{
		motor = new Jaguar(9);
		stick = new Joystick(1);
		compressor = new Compressor(1, 1);
		valve = new Solenoid(8);
		// Construct the dashboard sender object used to send hardware state
		// to the driver station
//		dds = new DashboardDataSender();
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Plyboy test code: 6:46PM 1/2/2012");
		dsLCD->UpdateLCD();
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		valve->Set(true);
		Wait(5.0);
		valve->Set(false);
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		
		GetWatchdog().SetEnabled(true);
		compressor->Start();
		
		GetWatchdog().SetExpiration(0.5);
		
		bool valve_state = false;
		
		while (IsOperatorControl())
		{
			motor->Set(stick->GetY());
			
			if (stick->GetRawButton(1) && !valve_state)
			{
				valve->Set(true);
				valve_state = true;
			}
			
			if (!stick->GetRawButton(1) && valve_state)
			{
				valve->Set(false);
				valve_state = false;
			}
			// Update driver station
			//dds->sendIOPortData(valve);

			GetWatchdog().Feed();
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

