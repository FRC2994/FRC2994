#include "WPILib.h"
#include "Gamepad.h"
#include "DashboardDataSender.h"
#include <math.h>

// Motor controller definitions
#define k_FrontLeftMotor 8
#define k_RearLeftMotor 5
#define k_FrontRightMotor 7
#define k_RearRightMotor 3

// Manipulator definitions
#define k_LiftMotor 2
#define k_LiftMotor2 10
#define k_ArmMotor 9
#define k_HandMotor 6

// Button definitions
#define k_HandOpenButton 5
#define k_HandCloseButton 7

#define k_MiniDeployButtonOne 9 
#define k_MiniDeployButtonTwo 10

// Driver station I/O assignments
#define k_dsUseSoftLimits 5
#define k_dsShowPosition 6
#define k_dsUseGyro 7
#define k_dsUseSquaredInputs 8

// Position Limits
#define k_ArmForwardPositionLimit 0.86
#define k_ArmReversePositionLimit 0.33
#define k_LiftForwardPositionLimit 6.85
#define k_LiftReversePositionLimit 9.78

class RobotDemo : public SimpleRobot
{
	RobotDrive * myRobot;   	// robot drive system
	Joystick   * leftJoyStick;	// joystick for x and y drive control
	Joystick   * rightJoyStick; // joystick for rotation drive control
	Gamepad    * gamePad;		// arm controller
	
	DashboardDataSender * dds;   // to enable real-time update of the hardware 
							     // state on the drive station
	DriverStationLCD *	 dsLCD;  // to enabl display of progress/debug info
								 // on the driver station LCD
	
	DriverStation * ds;			 // to enable reading of the driver station
								 // digital IO
	
	// The motor controllers for the drive and manipulator motors
	Jaguar * frontLeftMotor;
	Jaguar * rearLeftMotor;
	Jaguar * frontRightMotor;
	Jaguar * rearRightMotor;
	Jaguar * liftMotor;
	Jaguar * liftMotor2;
	Jaguar * armMotor;
	Jaguar * handMotor;

	Solenoid *miniDeploy;
	
	// Just to enable charging of the pressure cylinder
	Compressor * compressor;

	// A gyroscope to use for filed-centric mecanum drive		
	Gyro *gyro; 
	
public:
	RobotDemo(void)
	{
		// Construct the 4 motor controllers we need for the mecanum drive
		frontLeftMotor =  new Jaguar(k_FrontLeftMotor);
		rearLeftMotor =   new Jaguar(k_RearLeftMotor);
		frontRightMotor = new Jaguar(k_FrontRightMotor);
		rearRightMotor =  new Jaguar(k_RearRightMotor);
		
		// Now that the motors controllers are constructed, construct the
		// robot drive
		myRobot = new RobotDrive (frontLeftMotor, 
								  rearLeftMotor, 
								  frontRightMotor, 
								  rearRightMotor);
		
		// Construct the joysticks and gamepad we need to control the robot
		leftJoyStick  = new Joystick(1);
		rightJoyStick = new Joystick(2);
		gamePad       = new Gamepad(3);
		
		// Set up access to the driver station
		ds = DriverStation::GetInstance();
		
		// Construct the motor controller for the arm mechanism
		liftMotor = new Jaguar (k_LiftMotor); 
		liftMotor2 = new Jaguar (k_LiftMotor2); 
		
		// Construct the motor controller for the lift mechanism
		armMotor =  new Jaguar (k_ArmMotor);  
		
		// Assumption: the hand has a limit switch to define its 
		// initial position 		
		handMotor = new Jaguar (k_HandMotor);
		
		// Set up access to the driver station LCD
		dsLCD = DriverStationLCD::GetInstance();
				
		// Construct the dashboard sender object used to send hardware state
		// to the driver station
		dds = new DashboardDataSender();
		
		miniDeploy = new Solenoid(8);
		
		compressor = new Compressor (8, 8);
		
		// Construct the gyro we will use for field-centric mecanum drive
		gyro = new Gyro(1,1);

		// The right side motors spin in the opposite direction from
		// those on the left - the robot drive code needs to know this
		myRobot->SetInvertedMotor (RobotDrive::kFrontRightMotor, true);
		myRobot->SetInvertedMotor (RobotDrive::kRearRightMotor, true);
				
		// Let the user know which program is running
		dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "PWM-Joy-Opt-NoLine");
		dsLCD->UpdateLCD();
		
		GetWatchdog().SetExpiration(0.1);
	}

	float squared (float value)
	{
		float newValue = 0;
		if (fabs(value) < 0.05)
		{
			return 0;
		}
		
		newValue = value * value;
		
		if (value < 0.0)
		{
			newValue = -newValue;
		}
		return newValue;
	}
		
	void Autonomous(void)
	{
		GetWatchdog().SetEnabled(false);
		myRobot->Drive(0.2, 0.0); 	// drive forwards 20% speed
		Wait(2.0); 					// for 2 seconds
		myRobot->Drive(0.0, 0.0); 	// stop robot
	}

	void OperatorControl(void)
	{
		GetWatchdog().SetEnabled(false);

		// Starting the compressor doesn't so anything unless the external
		// compressor is attached to the robot
		//compressor->Start();
				
		float leftJoyStickXValue = 0.0;
		float leftJoyStickYValue = 0.0;
		float rightJoyStickXValue = 0.0;
		float gyroValue = 0.0;

		while (IsOperatorControl())
		{
			leftJoyStickXValue  = leftJoyStick->GetX();
			leftJoyStickYValue  = leftJoyStick->GetY();
			rightJoyStickXValue = rightJoyStick->GetX();
//
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3,"leftX: %4.2f",leftJoyStickXValue);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line4,"leftY: %4.2f",leftJoyStickYValue);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line5,"rghtX: %4.2f",rightJoyStickXValue);
			// If driver station digital input 8 is zero, square the mecanum drive inputs
			// to give finer low speed cotrol
			if (!ds->GetDigitalIn(k_dsUseSquaredInputs))
			{
				leftJoyStickXValue  = squared(leftJoyStickXValue);
				leftJoyStickYValue  = squared(leftJoyStickYValue);
				dsLCD->Printf(DriverStationLCD::kUser_Line3,14,"%4.2f",leftJoyStickXValue);
				dsLCD->Printf(DriverStationLCD::kUser_Line4,14,"%4.2f",leftJoyStickYValue);
				dsLCD->Printf(DriverStationLCD::kUser_Line5,14,"%4.2f",rightJoyStickXValue);
				rightJoyStickXValue = squared(rightJoyStickXValue);
			}
//
			// If driver station digital input is zero, enable use of the gyro for
			// field-centric driver mode
			if (!ds->GetDigitalIn(k_dsUseGyro))
			{
				gyroValue = gyro->GetAngle();
			}
			else
			{
				gyroValue = 0.0;
			}

			// Drive!
			myRobot->MecanumDrive_Cartesian(leftJoyStickXValue,
										    leftJoyStickYValue,
										    rightJoyStickXValue,
										    gyroValue);
			
			float leftGamePadStickValue = 0;
			float rightGamePadStickValue = 0;
			
			// Handle lift joystick on the game pad
			
			leftGamePadStickValue = -1.0 * gamePad->GetLeftY();
			if ((leftGamePadStickValue > 0.05) || (leftGamePadStickValue < -0.05))
			{
				liftMotor->Set(leftGamePadStickValue);
				liftMotor2->Set(leftGamePadStickValue);
			}
			else
			{
				liftMotor->Set(0);
				liftMotor2->Set(0);
			}
					
			// Handle arm joystick on the game pad - we need to dramatically slow
			// down the "down" movements due to the speedup caused by grvitational
			// "assist" and the up goes a bit to fast as well...
			
			rightGamePadStickValue = -1.0 * gamePad->GetRightY();
			if ((rightGamePadStickValue > 0.05) || (rightGamePadStickValue < -0.05))
			{
				if (rightGamePadStickValue > 0)
				{
					rightGamePadStickValue = rightGamePadStickValue/2;
				}
				else
				{
					rightGamePadStickValue = rightGamePadStickValue/16;
				}
					armMotor->Set(rightGamePadStickValue);
				}
			else
			{
				armMotor->Set(0);
			}
			
			//Handle manual opening/closing of the hand
			
			if(gamePad->GetRawButton(k_HandOpenButton))
			{
				//open the hand
				//dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "hand: open");
				handMotor->Set(.4);
			}
			else
			{
				if(gamePad->GetRawButton(k_HandCloseButton))
				{
					//dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "hand: close");
					//close the hand
					handMotor->Set(-0.4);
				}
				else
				{
					//stop motor
					//dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "hand:");
					handMotor->Set(0);
				}
			}
					
			// Handle deployment of the Minibot 
			
			if (gamePad->GetRawButton(k_MiniDeployButtonOne) && 
				gamePad->GetRawButton(k_MiniDeployButtonTwo))
			{
				miniDeploy->Set(true);
			} 
			else
			{
				miniDeploy->Set(false);
			}
			
			// Update driver station
			dds->sendIOPortData();

			if (!ds->GetDigitalIn(k_dsUseGyro))
			{
				dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "gyro: %6.4f", gyro->GetAngle());
			}
			dsLCD->UpdateLCD();
			GetWatchdog().Feed();
		}
	}		
};
	
	START_ROBOT_CLASS(RobotDemo);
	
