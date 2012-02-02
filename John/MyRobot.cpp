#include "WPILib.h"

#define kAngle_High 140
#define kAngle_Low 40

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive *my_robot; // robot drive system
	Joystick *right_stick;
	Joystick *left_stick;
	Jaguar *jag_left;
	Jaguar *jag_right;
	DriverStationLCD *dsLCD;
	Servo *drive_servos;

public:
	RobotDemo(void)
	{
		dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "John-Auto-mode 10:34AM 31/1/2012");
		dsLCD->UpdateLCD();
		left_stick = new Joystick(2);
		right_stick = new Joystick(1);
		jag_left = new Jaguar(8);
		jag_right = new Jaguar(7);
		drive_servos = new Servo(9);
		
		my_robot = new RobotDrive(jag_left, jag_right);
	}

	/** John's autonomous mode.
	 * This mode attempts to drive John forwards for two seconds at half speed, then for two seconds backwards at half speed.
	 * 
	 */
	void Autonomous(void)
	{
		jag_left->Set(0.5);
		jag_right->Set(0.5);
		Wait(0.005);
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Jag_left: %f", jag_left->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line5,1, "Jag_right: %f", jag_right->Get());
		dsLCD->UpdateLCD();
		Wait(2.0);
		jag_right->Set(0.5);
		jag_left->Set(0.5);
		Wait(0.005);
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Jag_left: %f", jag_left->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line5,1, "Jag_right: %f", jag_right->Get());
		dsLCD->UpdateLCD();
		Wait(2.0);
		jag_left->Set(0.0);
		jag_right->Set(0.0);
		Wait(0.005);
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Jag_left: %f", jag_left->Get());
		dsLCD->Printf(DriverStationLCD::kUser_Line5,1, "Jag_right: %f", jag_right->Get());
		dsLCD->UpdateLCD();
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
//		bool first = false;
//		int count = 0;
		
		drive_servos->SetAngle(90);
		
		bool arcade = false;
		bool left = false;
		
		while (IsOperatorControl())
		{
			// DEBUG: Useful for servo debugging!
//			if (++count >= 100)
//			{
//				if (right_stick->GetRawButton(2))
//				{
//					if (test_servo->GetAngle() > 0)
//					{
//						test_servo->SetAngle(test_servo->GetAngle() - 1);
//					}
//				}
//				if (right_stick->GetRawButton(3))
//				{
//					if (test_servo->GetAngle() < 150)
//					{
//						test_servo->SetAngle(test_servo->GetAngle() + 1);
//					}
//				}
//			}
			
			if (!arcade && right_stick->GetRawButton(10))
			{
				arcade = true;
			}
			else if (arcade && right_stick->GetRawButton(11))
			{
				arcade = false;
			}
			
			if (!left && right_stick->GetRawButton(8))
			{
				left = true;
			}
			else if (left && right_stick->GetRawButton(9))
			{
				left = false;
			}
			
			if (right_stick->GetRawButton(3))
			{
				drive_servos->SetAngle(kAngle_Low);
			}
			else if (right_stick->GetRawButton(2))
			{
				drive_servos->SetAngle(kAngle_High);
			}
			
//			if (!pos && right_stick->GetRawButton(2))
//			{
//				pos = true;
//				test_servo->SetAngle(0);
//			}
//			else if (pos && right_stick->GetRawButton(3))
//			{
//				pos = false;
//				test_servo->SetAngle(90);
//			}
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Left: %f, %f", left_stick->GetX(), left_stick->GetY());
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Right: %f, %f", right_stick->GetY(), right_stick->GetY());
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Arcade: %s", (arcade == true ? "arcade" : "tank"));
			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Hand: %s", (left == true ? "left" : "right"));
			dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Test servo: %f", drive_servos->GetAngle());
			dsLCD->UpdateLCD();
//			jag_left->Set(left_stick->GetY());
//			jag_right->Set(right_stick->GetY());
//			if (!first && right_stick->GetY() > 0.0)
//			{
//				first = true;
//				test_servo->SetAngle(90);
//			}
			if (arcade)
			{
				Joystick *current = left ? left_stick : right_stick;
				my_robot->ArcadeDrive(current->GetX(), current->GetY(), false);
			}
			else
			{
				jag_left->Set(left_stick->GetY());
				jag_right->Set(right_stick->GetY());
			}
			Wait(0.005);				// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(RobotDemo);
