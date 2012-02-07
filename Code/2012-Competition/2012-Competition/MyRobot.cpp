#include "WPILib.h"
#include "Gamepad.h"

#define ButtonEvent(i) m_button_changes[(i)]
#define ButtonState(i) m_button_current[(i)]
#define SwitchEvent(i) m_switch_changes[(i)]
#define SwitchState(i) m_switch_current[(i)]

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	typedef enum {no, off, on} changes;
	typedef enum {s1, s2, s3, s4, s5, s6, s7, s8, SWITCH_ARR_SIZE} switches;
	typedef enum {b_shoot, b_dzero, b_dset_small, b_dset_large, b_inc_large, b_inc_small, b_dec_large, b_dec_small, BUTTON_ARR_SIZE} buttons;
public:
	int m_distance;

	bool m_button_current[BUTTON_ARR_SIZE];
	bool m_button_previous[BUTTON_ARR_SIZE];
	changes m_button_changes[BUTTON_ARR_SIZE];

	bool m_switch_current[SWITCH_ARR_SIZE];
	bool m_switch_previous[SWITCH_ARR_SIZE];
	changes m_switch_changes[SWITCH_ARR_SIZE];

	Gamepad *m_gamepad;
	DriverStationLCD *m_ds_lcd;
	Joystick *m_left_joy;
	DriverStation *ds;
	//	DashboardDataSender *dds;

	RobotDemo(void) : m_distance(0)
	{
		//		dds = new DashboardDataSender();
		m_left_joy = new Joystick(2);
		m_gamepad = new Gamepad(3);
		m_ds_lcd = DriverStationLCD::GetInstance();
		ds = DriverStation::GetInstance();
		
		m_ds_lcd->PrintfLine(DriverStationLCD::kUser_Line1, "Plyboy 7:20PM state test");
		m_ds_lcd->UpdateLCD();
	}

	bool GetSwitch(int i)
	{
		/*
		 * TODO: Ken and Gabby code your own version of this when the actual production code is required. 
		 * All this function needs to do is return what the current value is for a switch i.-Jack
		 */
		return m_left_joy->GetRawButton(i+1);
	}

	void ProcessType(int i, bool *current, bool *prev, changes *changes)
	{
		prev[i] = current[i];
		current[i] = (current == m_button_current ? m_gamepad->GetRawButton(i+1) : GetSwitch(i));
		if (current[i] == prev[i])
		{
			changes[i] = no;
		}
		else
		{
			changes[i] = (current[i] ? on : off);
		}
	}

	void RetriveButton(int i)
	{
		ProcessType(i, m_button_current, m_button_previous, m_button_changes);
	}

	void RetriveSwitch(int i)
	{
		ProcessType(i, m_switch_current, m_switch_previous, m_switch_changes);
	}

	void GetSwitches(void)
	{
		for (int i = 0; i < SWITCH_ARR_SIZE; i++)
		{
			RetriveSwitch(i);
		}
	}

	void GetButtons(void)
	{
		for (int i = 0; i < BUTTON_ARR_SIZE; i++)
		{
			RetriveButton(i);
		}
	}

	void InitSwitches(void)
	{
		for (int i = 0; i < SWITCH_ARR_SIZE; i++)
		{
			m_switch_current[i] = false;
			m_switch_previous[i] = false;
			m_switch_changes[i] = no;
		}
	}

	void InitButtons(void)
	{
		for (int i = 0; i < BUTTON_ARR_SIZE; i++)
		{
			m_button_current[i] = false;
			m_button_previous[i] = false;
			m_button_changes[i] = no;
		}
	}

	void HandleShooterInputs(void)
	{
		/*
		 * TODO: Ken and Gabby: You two have been, to my knowledge, been working on something to do with the control modes of the robot (correct me if I'm wrong).
		 * I don't want to reinvent the wheel, so I'm leaving this for you guys to plug your code into.-Jack
		 */
		if (ButtonEvent(b_shoot) == on)
		{
			m_ds_lcd->PrintfLine(DriverStationLCD::kUser_Line5, "Shoot!");
			m_ds_lcd->UpdateLCD();
		}
		else if (ButtonEvent(b_dzero) == on)
		{
			m_distance = 0;
		}
		else if (ButtonEvent(b_dset_small) == on)
		{
			m_distance = 10;
		}
		else if (ButtonEvent(b_dset_large) == on)
		{
			m_distance = 40;
		}
		else if (ButtonEvent(b_inc_large) == on)
		{
			m_distance += 5;
		}
		else if (ButtonEvent(b_dec_large) == on)
		{
			if (m_distance >= 5)
			{
				m_distance -= 5;
			}
			else
			{
				m_distance = 0;
			}
		}
		else if (ButtonEvent(b_inc_small) == on)
		{
			m_distance += 1;
		}
		else if (ButtonEvent(b_dec_small) == on && m_distance != 0)
		{
			m_distance -= 1;
		}
	}
	
	void DebugPrint()
	{
		for (int i = 0; i < BUTTON_ARR_SIZE; i++)
		{
			if (m_button_changes[i] == on || m_button_changes[i] == off)
			{
				m_ds_lcd->PrintfLine(DriverStationLCD::kUser_Line6, "Gampad %d from %d to %d", i+1, m_button_previous[i], m_button_current[i]);
			}
		}

		for (int i = 0; i < SWITCH_ARR_SIZE; i++)
		{
			if (m_switch_changes[i] == on || m_switch_changes[i] == off)
			{
				m_ds_lcd->PrintfLine(DriverStationLCD::kUser_Line5, "Switch %d from %d to %d", i+1, m_switch_previous[i], m_switch_current[i]);\
			}
		}
		m_ds_lcd->UpdateLCD();
	}
	
	
	//TODO: This function needs a better name!-Jack
	void PrintDriverStation(void)
	{
		m_ds_lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Shooting distance: %d", m_distance);
		m_ds_lcd->UpdateLCD();
	}
	
	void RunBallCollector(void)
	{
		/* TODO: Gabby and Ken: Integrate your ball collection state machine here! */
	}

	void OperatorControl(void)
	{
		InitButtons();
		InitSwitches();
		while (IsOperatorControl())
		{
			// Get inputs.
			GetButtons();
			GetSwitches();
			
			// Process inputs.
			// TODO: Have a talk about where we're going to put stuff on the digital sidecar so that I/anyone else can implement these methods. They are dependent on these factors.-Jack
//			HandleDriverInputs();
//			HandleArmInput();
			HandleShooterInputs();
			
			RunBallCollector();
			
			if (!ds->GetDigitalIn(1))
			{
				DebugPrint();
			}
			
			PrintDriverStation();
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

