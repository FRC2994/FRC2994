#include "WPILib.h"
#include "Gamepad.h"
#include "DashboardDataSender.h"

#define Button(i) m_button_changes[((i) - 1)]
#define BUTTON_ARR_SIZE 8

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	typedef enum {no, off, on} changes;
public:
	int distance;
	bool m_button_current[BUTTON_ARR_SIZE];
	bool m_button_previous[BUTTON_ARR_SIZE];
	changes m_button_changes[BUTTON_ARR_SIZE];
	Gamepad *gamepad;
	DriverStationLCD *ds_lcd;
	Jaguar *jag_arm;
	Joystick *joy;
	DashboardDataSender *dds;
	
	RobotDemo(void) : distance(0)
	{
//		dds = new DashboardDataSender();
		jag_arm = new Jaguar(9);
		joy = new Joystick(1);
		for (int i = 0; i < BUTTON_ARR_SIZE; i++)
		{
			m_button_current[i] = false;
			m_button_previous[i] = false;
			m_button_changes[i] = no;
		}
		gamepad = new Gamepad(3);
		ds_lcd = DriverStationLCD::GetInstance();
		ds_lcd->PrintfLine(DriverStationLCD::kUser_Line1, "Plyboy gamepad 11:52AM 4/2/12");
		ds_lcd->UpdateLCD();
	}
	
	void ProcessSwitch(int i)
	{
		m_button_previous[i] = m_button_current[i];
		m_button_current[i] = gamepad->GetRawButton(i+1);
		if (m_button_current[i] == m_button_previous[i])
		{
			m_button_changes[i] = no;
		}
		else
		{
			m_button_changes[i] = (m_button_current[i] ? on : off);
		}
	}
	
	void ProcessSwitches(void)
	{
		for (int i = 0; i < BUTTON_ARR_SIZE; i++)
		{
			ProcessSwitch(i);
		}
	}
	
	void OperatorControl(void)
	{
		while (IsOperatorControl())
		{
			ProcessSwitches();
			
			if (Button(1) == on)
			{
				ds_lcd->PrintfLine(DriverStationLCD::kUser_Line5, "Shoot!");
				ds_lcd->UpdateLCD();
			}
			if (Button(2) == on)
			{
				distance = 0;
			}
			else if (Button(3) == on)
			{
				distance = 10;
			}
			else if (Button(4) == on)
			{
				distance = 40;
			}
			else if (Button(5) == on)
			{
				distance += 5;
			}
			else if (Button(7) == on)
			{
				if (distance >= 5)
				{
					distance -= 5;
				}
				else
				{
					distance = 0;
				}
			}
			else if (Button(6) == on)
			{
				distance += 1;
			}
			else if (Button(8) == on && distance != 0)
			{
				distance -= 1;
			}
			
			for (int i = 0; i < BUTTON_ARR_SIZE; i++)
			{
				if (m_button_changes[i] == on || m_button_changes[i] == off)
				{
					ds_lcd->PrintfLine(DriverStationLCD::kUser_Line6, "Gampad %d from %d to %d", i+1, m_button_previous[i], m_button_current[i]);
					ds_lcd->UpdateLCD();
				}
			}
			
			ds_lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Shooting distance: %d", distance);
			ds_lcd->UpdateLCD();
			jag_arm->Set(joy->GetY());
//			dds->sendIOPortData(NULL);
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

