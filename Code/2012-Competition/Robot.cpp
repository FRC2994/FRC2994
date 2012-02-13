#include "WPILib.h"
#include "Gamepad.h"
#include "DashboardDataFormat.h"

#define ButtonEvent(i) m_gamepad_changes[(i)]
#define ButtonState(i) m_gamepad_current[(i)]
#define SwitchEvent(i) m_collector_changes[(i)]
#define SwitchState(i) m_collector_current[(i)]
#define LJoyEvent(i) m_left_joystick_changes[(i)]
#define LJoyState(i) m_left_joystick_current[(i)]
#define RJoyEvent(i) m_right_joystick_changes[(i)]
#define RJoyState(i) m_right_joystick_current[(i)]
#define EitherJoyPressed(i) (m_right_joystick_changes[(i)] == pressed || m_left_joystick_changes[(i)] == pressed)

// Module Assignements

// Analog Module
#define ANALOG_S1 1
#define ANALOG_S2 2
#define ANALOG_S3 3
#define ANALOG_S4 4

// Digital IOs
#define LEFT_DRIVE_ENCODER_A 			1
#define LEFT_DRIVE_ENCODER_B 			2
#define RIGHT_DRIVE_ENCODER_A			3
#define RIGHT_DRIVE_ENCODER_B			4
#define TOP_SHOOTER_ENCODER_A			5
#define TOP_SHOOTER_ENCODER_B			6
#define BOTTOM_SHOOTER_ENCODER_A		7
#define BOTTOM_SHOOTER_ENCODER_B		8
#define SHOOTER_AZIUMTH_ENCODER_A	9
#define SHOOTER_AZIMUTH_ENCODER_B	10
#define ULTRASONIC_ENABLE				11
#define ULTRASONIC_INPUT				12
#define CAMERA_LIGHT_ENABLE			13
#define PNEUMATIC_PRESSURE_SWITCH	14

// PWM
#define LEFT_DRIVE			1
#define RIGHT_DRIVE			2
#define LEFT_SHIFT			4
#define RIGHT_SHIFT			5
#define CAMERA					6
#define ELEVATION_MOTOR		7
#define SHOOTER_AZIMUTH		8

// Relay
#define ARM_MOTOR				1
#define COMPRESSOR			2
#define BALL_DISPLAY_0		3
#define BALL_DISPLAY_1		4
#define BALL_COLLECTOR_M1	6
#define BALL_COLLECTOR_M2	7
#define BALL_COLLECTOR_M3	8

// Solenoid
#define SHOOTER_ELEVATION	8

// Digital IO Assignments
#define DRIVE_TYPE 				1
#define ARCADE_DRIVE_JOYSTICK 2
#define INITIAL_POSITION_0		3
#define INITIAL_POSITION_1		4
#define BASKET_CHOICE_0			5
#define BASKET_CHOICE_1			6
#define DEBUG 						8

class Robot2012 : public SimpleRobot
{
	// Objects

	// Input Devices

	// Human Input
	Gamepad *gamepad;
	Joystick *leftJoystick;
	Joystick *rightJoystick;

	// Sensor Input
	AnalogTrigger *switch_1;
	AnalogTrigger *switch_2;
	AnalogTrigger *switch_3;
	AnalogTrigger *switch_4;
	Encoder *leftDriveEncoder;
	Encoder *rightDriveEncoder;
	Encoder *bottomShooterEncoder;
	Encoder *topShooterEncoder;
	Encoder *shooterAzimuthEncoder;
	Ultrasonic *ultrasonicSensor;
	DigitalOutput *camera_light;


	// Motor Controllers
	Jaguar *driveLeftMotor;
	Jaguar *driveRightMotor;
	Jaguar *shooterBottomMotor;
	Jaguar *shooterTopMotor;
	Jaguar *shooterAzimuthMotor;
	Jaguar *shooterElevationMotor;

	// Servos
	Servo *leftShifter;
	Servo *rightShifter;
	Servo *cameraServo;

	// Motor Relays
	Relay *ballCollectorM1;
	Relay *ballCollectorM2;
	Relay *ballCollectorM3;
	Relay *armMotor;
	Relay *ballDisplay_0;
	Relay *ballDisplay_1;

	// Misc
	RobotDrive 				*robotDrive;
	DashboardDataFormat 	*dds;
	DriverStationLCD 		*dsLCD;
	DriverStation 			*ds;
	Timer 					*ballCollectorTimer;
	Compressor				*compressor;
	Solenoid						*shooterElevationValve;
	
	
	// Motors 
	typedef enum {motor_off, motor_fwd, motor_rev} motor_states;
	typedef enum {m1, m2, m3, m4, NUM_BALL_COLLECTOR_MOTORS} motors;
	motor_states m_motorState[NUM_BALL_COLLECTOR_MOTORS];

	typedef enum {I, C, D, S, W, F} collector_modes;
	collector_modes m_collectorMode;

	UINT32 m_ballCount;
	float m_bottomShooterMotorSetting;
	float m_topShooterMotorSetting;

	// gamepad (real) button assignments
#define SHOOT 			1
#define LOW				2
#define MEDIUM			3
#define HIGH			4
#define INCR_SMALL 	5
#define INCR_BIG		6
#define DECR_SMALL	7
#define DECR_BIG		8
	
	typedef enum {none, released, pressed} changes;
	typedef enum {off, on} state;
	
	// gamepad abstract arrays
	typedef enum {shoot, low, medium, high, incr_small, incr_big, decr_small,
		decr_big, GAMEPAD_ARRAY_SIZE} gamepad_buttons;

		state   m_gamepad_current[GAMEPAD_ARRAY_SIZE];
		state   m_gamepad_previous[GAMEPAD_ARRAY_SIZE];
		changes m_gamepad_changes[GAMEPAD_ARRAY_SIZE];

#define GamepadButtonState(i) m_gamepad_current[(i)]
#define GamepadButtonEvent(i) m_gamepad_changes[(i)]
		
		Robot2012(void)
		{
			//TODO: Implement the initialization of all the objects above using Ken's constants.
			gamepad = new Gamepad(3);
			leftJoystick = new Joystick(2);
			rightJoystick = new Joystick(1);
			
			switch_1 = new AnalogTrigger(ANALOG_S1);
			switch_2 = new AnalogTrigger(ANALOG_S2);
			switch_3 = new AnalogTrigger(ANALOG_S3);
			switch_4 = new AnalogTrigger(ANALOG_S4);
			
			leftDriveEncoder = new Encoder(LEFT_DRIVE_ENCODER_A, LEFT_DRIVE_ENCODER_B);
			rightDriveEncoder = new Encoder(RIGHT_DRIVE_ENCODER_A, RIGHT_DRIVE_ENCODER_B);
			shooterAzimuthEncoder = new Encoder(SHOOTER_AZIUMTH_ENCODER_A, SHOOTER_AZIMUTH_ENCODER_B);
			
			driveLeftMotor = new Jaguar(LEFT_DRIVE);
			driveRightMotor = new Jaguar(RIGHT_DRIVE);
			//TODO: We need something for the shooter top and bottom. Ken: Can you explain what these motors do and their positioning on the sidecar?
			shooterAzimuthMotor = new Jaguar(SHOOTER_AZIMUTH);
			shooterElevationMotor = new Jaguar(ELEVATION_MOTOR);
			
			shooterElevationValve = new Solenoid(SHOOTER_ELEVATION);
			
			leftShifter = new Servo(LEFT_SHIFT);
			rightShifter = new Servo(RIGHT_SHIFT);
			cameraServo = new Servo(CAMERA);
			
			ds = DriverStation::GetInstance();
			dsLCD = DriverStationLCD::GetInstance();
			
			robotDrive = new RobotDrive(driveLeftMotor, driveRightMotor);
			
			dds = new DashboardDataFormat();
			
			compressor = new Compressor(PNEUMATIC_PRESSURE_SWITCH, COMPRESSOR);
			
			camera_light = new DigitalOutput(CAMERA_LIGHT_ENABLE);
			
			ultrasonicSensor = new Ultrasonic(ULTRASONIC_ENABLE, ULTRASONIC_INPUT);
			
			ballCollectorTimer = new Timer();
		}

		void InitGamepadButtons (void)
		{
			for (int i=0; i<GAMEPAD_ARRAY_SIZE; i++)
			{
				m_gamepad_current[i] = off;
				m_gamepad_previous[i] = off;
				m_gamepad_changes[i] = none;
			}
		}
		
		void ProcessChanges(state *current, state *previous, changes *change, int size)
		{
			int i = 0;
			for (i = 0; i < size; i++)
			{
				if (current[i] == previous[i])
				{
					change[i] = none;
				}
				else
				{
					change[i] = (current[i] ? pressed : released);
				}
				previous[i] = current[i];
			}
		}

		void GetGamepadButtons (void)
		{
			m_gamepad_current[shoot] = gamepad->GetRawButton(SHOOT) ? on : off;
			m_gamepad_current[low] 	= gamepad->GetRawButton(LOW) ? on : off;
			m_gamepad_current[medium] = gamepad->GetRawButton(MEDIUM) ? on : off;
			m_gamepad_current[high] = gamepad->GetRawButton(HIGH) ? on : off;
			m_gamepad_current[incr_small] = gamepad->GetRawButton(INCR_SMALL) ? on : off;
			m_gamepad_current[incr_big] = gamepad->GetRawButton(INCR_BIG) ? on : off;
			m_gamepad_current[decr_small] = gamepad->GetRawButton(DECR_SMALL) ? on : off;
			m_gamepad_current[decr_big] = gamepad->GetRawButton(DECR_BIG) ? on : off;

			ProcessChanges(m_gamepad_current,
					m_gamepad_previous,
					m_gamepad_changes,
					GAMEPAD_ARRAY_SIZE);
		}

		// left joystick (real) button assignments
#define SHIFTER 	1
#define M1_FWD		6
#define M1_REV		7
#define M2_FWD		8
#define M2_REV		9
#define M3_FWD		11
#define M3_REV		10

		// left joystick abstract arrays
		typedef enum {shift, m1_fwd, m1_rev, m2_fwd, m2_rev, m3_fwd, m3_rev,
			LEFT_JOYSTICK_ARRAY_SIZE} left_joystick_buttons;

			state   m_left_joystick_current[LEFT_JOYSTICK_ARRAY_SIZE];
			state   m_left_joystick_previous[LEFT_JOYSTICK_ARRAY_SIZE];
			changes m_left_joystick_changes[LEFT_JOYSTICK_ARRAY_SIZE];

#define LeftJoystickButtonState(i) m_left_joystick_current[(i)]
#define LeftJoystickButtonEvent(i) m_left_joystick_changes[(i)]

			void InitLeftJoystickButtons (void)
			{
				for (int i=0; i<LEFT_JOYSTICK_ARRAY_SIZE; i++)
				{
					m_left_joystick_current[i] = off;
					m_left_joystick_previous[i] = off;
					m_left_joystick_changes[i] = none;
				}
			}

			void GetLeftJoystickButtons (void)
			{
				m_left_joystick_current[shift]  = leftJoystick->GetRawButton(SHIFTER) ? on : off;
				m_left_joystick_current[m1_fwd] = leftJoystick->GetRawButton(M1_FWD) ? on : off;
				m_left_joystick_current[m1_rev] = leftJoystick->GetRawButton(M1_REV) ? on : off;
				m_left_joystick_current[m2_fwd] = leftJoystick->GetRawButton(M2_FWD) ? on : off;
				m_left_joystick_current[m2_rev] = leftJoystick->GetRawButton(M2_REV) ? on : off;
				m_left_joystick_current[m3_fwd] = leftJoystick->GetRawButton(M3_FWD) ? on : off;
				m_left_joystick_current[m3_rev] = leftJoystick->GetRawButton(M3_REV) ? on : off;

				ProcessChanges(m_left_joystick_current,
						m_left_joystick_previous,
						m_left_joystick_changes,
						LEFT_JOYSTICK_ARRAY_SIZE);

			}

			// right joystick (real) button assignments
#define FLUSH_BALL_COLLECTOR  6
#define ENABLE_BALL_COLLECTOR 7

			// right joystick abstract arrays
			typedef enum {flush, enable, RIGHT_JOYSTICK_ARRAY_SIZE} right_joystick_buttons;

			state   m_right_joystick_current[RIGHT_JOYSTICK_ARRAY_SIZE];
			state   m_right_joystick_previous[RIGHT_JOYSTICK_ARRAY_SIZE];
			changes m_right_joystick_changes[RIGHT_JOYSTICK_ARRAY_SIZE];

#define RightJoystickButtonState(i) m_right_joystick_current[(i)]
#define RightJoystickButtonEvent(i) m_right_joystick_changes[(i)]

			void InitRightJoystickButtons (void)
			{
				for (int i=0; i<RIGHT_JOYSTICK_ARRAY_SIZE; i++)
				{
					m_right_joystick_current[i] = off;
					m_right_joystick_previous[i] = off;
					m_right_joystick_changes[i] = none;
				}
			}

			void GetRightJoystickButtons (void)
			{
				m_right_joystick_current[shift]  = rightJoystick->GetRawButton(SHIFTER) ? on : off;
				m_right_joystick_current[m1_fwd] = rightJoystick->GetRawButton(M1_FWD) ? on : off;
				m_right_joystick_current[m1_rev] = rightJoystick->GetRawButton(M1_REV) ? on : off;
				m_right_joystick_current[m2_fwd] = rightJoystick->GetRawButton(M2_FWD) ? on : off;
				m_right_joystick_current[m2_rev] = rightJoystick->GetRawButton(M2_REV) ? on : off;
				m_right_joystick_current[m3_fwd] = rightJoystick->GetRawButton(M3_FWD) ? on : off;
				m_right_joystick_current[m3_rev] = rightJoystick->GetRawButton(M3_REV) ? on : off;

				ProcessChanges(m_right_joystick_current,
						m_right_joystick_previous,
						m_right_joystick_changes,
						RIGHT_JOYSTICK_ARRAY_SIZE);

			}

			// analog (real) switch assignments
#define S1 1
#define S2 2
#define S3 3
#define S4 4

			// analog switch arrays
			typedef enum {s1, s2, s3, s4, COLLECTOR_SWITCH_ARRAY_SIZE} analog_switches;

			state m_collector_current[COLLECTOR_SWITCH_ARRAY_SIZE];
			state m_collector_previous[COLLECTOR_SWITCH_ARRAY_SIZE];
			changes m_collector_changes[COLLECTOR_SWITCH_ARRAY_SIZE];

#define CollectorSwitchState(i) m_collector_current[(i)]
#define CollectorSwitchEvent(i) m_collector_changes[(i)]

			void InitCollectorSwitches (void)
			{
				for (int i=0; i<COLLECTOR_SWITCH_ARRAY_SIZE; i++)
				{
					m_collector_current[i] = off;
					m_collector_previous[i] = off;
					m_collector_changes[i] = none;
				}
			}

			void GetCollectorSwitches(void)
			{
				m_collector_current[s1] = switch_1->GetTriggerState() ? on : off;
				m_collector_current[s2] = switch_2->GetTriggerState() ? on : off;
				m_collector_current[s3] = switch_3->GetTriggerState() ? on : off;
				m_collector_current[s4] = switch_4->GetTriggerState() ? on : off;

				ProcessChanges(m_collector_current,
						m_collector_previous,
						m_collector_changes,
						COLLECTOR_SWITCH_ARRAY_SIZE);	
			}
			
			void HandleArm(void)
			{
				static bool arm_up = off;
				if (arm_up && EitherJoyPressed(7))
				{
					arm_up = true;
					armMotor->Set(Relay::kForward);
				}
				if (!arm_up && EitherJoyPressed(8))
				{
					arm_up = false;
					armMotor->Set(Relay::kReverse);
				}
			}
			
			void HandleDriverInputs(void)
			{
				Joystick *current = m_ds->GetDigitalIn(3) ? rightJoystick : leftJoystick;
				
				if (EitherJoyPressed(3))
				{
					leftShifter->SetAngle(40);
					rightShifter->SetAngle(40);
				}
				if (EitherJoyPressed(2))
				{
					leftShifter->SetAngle(140);
					rightShifter->SetAngle(140);
				}
				
				//TODO: Does this work?
				if (!m_ds->GetDigitalIn(2))
				{
					robotDrive->ArcadeDrive(current);
				}
				else
				{
					robotDrive->TankDrive(leftJoystick, rightJoystick);
				}
			}
			
			void HandleShooterInputs(void)
			{
				//TODO: Empty block.
			}
			
			void RunBallCollectorStateMachine(void)
			{
				//TODO: Ken and Gabby: Stick state machine body in here.
			}

			// Main loop
			void OperatorControl(void)
			{
				while (IsOperatorControl())
				{
					// Get inputs that we need to know both current and previous state
					GetGamepadButtons();
					GetLeftJoystickButtons ();
					GetRightJoystickButtons ();
					GetCollectorSwitches();
	
	
					// Make any necessary changes to the arm
					HandleArm ();
	
					// Drive the robot
					HandleDriverInputs();
	
					// Process the shooter button and joystick inputs. This will result
					// in, where appropriate:
					// - camera postion changes
					// - shooter azimuth changes
					// - target distance recomnputations (based upon manual and automatic
					//   inputs)
					HandleShooterInputs();
	
					// Handle the collection of balls from the floor automatically
					RunBallCollectorStateMachine ();
				}
			}
};
