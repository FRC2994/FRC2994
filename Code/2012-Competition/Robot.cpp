#include "WPILib.h"
#include "Gamepad.h"
#include "DashboardDataFormat.h"

// Access macros
#define CollectorSwitchState(i) m_collector_current[(i)]
#define CollectorSwitchEvent(i) m_collector_changes[(i)]
#define EitherJoystickButtonEvent(i) (m_right_joystick_changes[(i)] == pressed || m_left_joystick_changes[(i)] == pressed)
#define GamepadButtonState(i) m_gamepad_current[(i)]
#define GamepadButtonEvent(i) m_gamepad_changes[(i)]
#define LeftJoystickButtonState(i) m_left_joystick_current[(i)]
#define LeftJoystickButtonEvent(i) m_left_joystick_changes[(i)]
#define RightJoystickButtonState(i) m_right_joystick_current[(i)]
#define RightJoystickButtonEvent(i) m_right_joystick_changes[(i)]

// Driver station digital Inputs
#define DS_LEFT_OR_RIGHT_STICK	3
#define DS_DRIVE_TYPE			2

// Module Assignements

// Analog Module
#define ANALOG_S1 1
#define ANALOG_S2 2
#define ANALOG_S3 3
#define ANALOG_S4 4

// Digital IOs
#define LEFT_DRIVE_ENCODER_A		1
#define LEFT_DRIVE_ENCODER_B 		2
#define RIGHT_DRIVE_ENCODER_A		3
#define RIGHT_DRIVE_ENCODER_B		4
#define TOP_SHOOTER_ENCODER_A		5
#define TOP_SHOOTER_ENCODER_B		6
#define BOTTOM_SHOOTER_ENCODER_A	7
#define BOTTOM_SHOOTER_ENCODER_B	8
#define SHOOTER_AZIUMTH_ENCODER_A	9
#define SHOOTER_AZIMUTH_ENCODER_B	10
#define ULTRASONIC_ENABLE			11
#define ULTRASONIC_INPUT			12
#define CAMERA_LIGHT_ENABLE			13
#define PNEUMATIC_PRESSURE_SWITCH	14

// PWM
#define LEFT_DRIVE_MOTOR		1
#define RIGHT_DRIVE_MOTOR		2
#define ARM_MOTOR				3
#define LEFT_SHIFT_SERVO		4
#define RIGHT_SHIFT_SERVO		5
#define CAMERA_SERVO			6
#define ELEVATION_MOTOR			7
#define SHOOTER_AZIMUTH_MOTOR	8
#define BOTTOM_SHOOTER_MOTOR	9
#define TOP_SHOOTER_MOTOR		10

// Relays
#define COMPRESSOR			2
#define BALL_DISPLAY_0		3
#define BALL_DISPLAY_1		4
#define BALL_COLLECTOR_M1	6
#define BALL_COLLECTOR_M2	7
#define BALL_COLLECTOR_M3	8

// Solenoids
#define SHOOTER_ELEVATION_SOLENOID	8

// Digital IO Assignments
#define DRIVE_TYPE 				1
#define ARCADE_DRIVE_JOYSTICK 	2
#define INITIAL_POSITION_0		3
#define INITIAL_POSITION_1		4
#define BASKET_CHOICE_0			5
#define BASKET_CHOICE_1			6
#define DEBUG 					8

// gamepad (real) button assignments
#define SHOOT 		1
#define LOW			2
#define MEDIUM		3
#define HIGH		4
#define INCR_SMALL 	5
#define INCR_BIG	6
#define DECR_SMALL	7
#define DECR_BIG	8

// left joystick (real) button assignments
#define SHIFTER 	1
#define M1_FWD		6
#define M1_REV		7
#define M2_FWD		8
#define M2_REV		9
#define M3_FWD		11
#define M3_REV		10

// right joystick (real) button assignments
#define FLUSH_BALL_COLLECTOR  6
#define ENABLE_BALL_COLLECTOR 7

// analog (real) switch assignments
#define S1 1
#define S2 2
#define S3 3
#define S4 4

// Human input devices
#define RIGHT_JOYSTICK	1
#define LEFT_JOYSTICK	2
#define GAMEPAD			3

// Types used for monitoring buttons and switches
typedef enum {none, released, pressed} changes;
typedef enum {off, on} state;

// Ball collector motors
typedef enum {motor_off, motor_fwd, motor_rev} motor_states;
const char *motorStateStrings[] = {"O", "F", "R"};
typedef enum {m1, m2, m3, m4, NUM_BALL_COLLECTOR_MOTORS} motors;

// Ball colllector states
typedef enum {I, C, D, S, W, F} collector_modes;
char *collectorModeLetters[] = {"I", "C", "D", "S", "W", "F"};

// Structure for shooter tables
#define SHOOTER_TABLE_ENTRIES 5  // this will have to be larger
typedef struct {
	float bottomMotorSetting;
	float topMotorSetting;
	UINT32 bottomDesriedRPM;
	UINT32 topDesiredRPM;
} shooter_table;

const shooter_table m_lowerBasketTable[SHOOTER_TABLE_ENTRIES] =
		{{0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		};
const shooter_table 	m_middleBasketTable[SHOOTER_TABLE_ENTRIES] =
		{{0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		};

const shooter_table 	m_upperBasketTable[SHOOTER_TABLE_ENTRIES] =
		{{0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		 {0.0, 0.0, 0,0},
		};


class Robot2012 : public SimpleRobot
{	
	// Input Devices

	// Human Input
	Gamepad 	*gamepad;
	Joystick 	*leftJoystick;
	Joystick 	*rightJoystick;

	// Sensor Input
	AnalogTrigger 	*switch_1;
	AnalogTrigger 	*switch_2;
	AnalogTrigger 	*switch_3;
	AnalogTrigger 	*switch_4;
	Encoder 		*leftDriveEncoder;
	Encoder 		*rightDriveEncoder;
	Encoder 		*bottomShooterEncoder;
	Encoder 		*topShooterEncoder;
	Encoder 		*shooterAzimuthEncoder;
	Ultrasonic 		*ultrasonicSensor;
	DigitalOutput 	*camera_light;

	// Motor Controllers
	Jaguar *driveLeftMotor;
	Jaguar *driveRightMotor;
	Jaguar *shooterBottomMotor;
	Jaguar *shooterTopMotor;
	Jaguar *shooterAzimuthMotor;
	Jaguar *shooterElevationMotor;
	Jaguar *armMotor;

	// Servos
	Servo *leftShifter;
	Servo *rightShifter;
	Servo *cameraServo;

	// Motor Relays
	Relay *ballCollectorM1;
	Relay *ballCollectorM2;
	Relay *ballCollectorM3;
	Relay *ballDisplay_0;
	Relay *ballDisplay_1;

	// Misc
	RobotDrive 			*robotDrive;
	DashboardDataFormat *dds;
	DriverStationLCD 	*dsLCD;
	DriverStation 		*ds;
	Timer 				*ballCollectorTimer;
	Compressor			*compressor;
	Solenoid			*shooterElevationValve;
	
	// Ball collector Motors 
	motor_states m_motorState[NUM_BALL_COLLECTOR_MOTORS];

	// Ball collector state
	collector_modes m_collectorMode;
	UINT32 m_ballCount;
	
	// Analog switch arrays (for ball collector)
	typedef enum {s1, s2, s3, s4, COLLECTOR_SWITCH_ARRAY_SIZE} analog_switches;
	
	state m_collector_current[COLLECTOR_SWITCH_ARRAY_SIZE];
	state m_collector_previous[COLLECTOR_SWITCH_ARRAY_SIZE];
	changes m_collector_changes[COLLECTOR_SWITCH_ARRAY_SIZE];

	// Shooter settings
	typedef enum {basket_low, basket_medium, basket_high,
				  SHOOTER_HEIGHT_ARRAY_SIZE} basket_height;
	basket_height m_targetBasketHeight;
	
	UINT32 	m_distance;

	float  	m_shooterBottomScaleFactor;
	float  	m_shooterTopScaleFactor;
	float 	m_bottomShooterMotorSetting;
	float 	m_topShooterMotorSetting;
	UINT32 	m_bottomShooterDesiredRPM;
	UINT32 	m_topShooterDesiredRPM;
	
	// We keep wasily indexable (and extensible) arrays for only
	// those buttons and switches that we want to monitor. These are
	// updated at the start of each pass through the main loop
	
	// gamepad abstract arrays
	typedef enum {shoot, low, medium, high, incr_small, incr_big, decr_small,
	decr_big, GAMEPAD_ARRAY_SIZE} gamepad_buttons;

	state   m_gamepad_current[GAMEPAD_ARRAY_SIZE];
	state   m_gamepad_previous[GAMEPAD_ARRAY_SIZE];
	changes m_gamepad_changes[GAMEPAD_ARRAY_SIZE];
	
	// left joystick abstract arrays
	typedef enum {shift, m1_fwd, m1_rev, m2_fwd, m2_rev, m3_fwd, m3_rev,
	LEFT_JOYSTICK_ARRAY_SIZE} left_joystick_buttons;

	state   m_left_joystick_current[LEFT_JOYSTICK_ARRAY_SIZE];
	state   m_left_joystick_previous[LEFT_JOYSTICK_ARRAY_SIZE];
	changes m_left_joystick_changes[LEFT_JOYSTICK_ARRAY_SIZE];
	
	// right joystick abstract arrays
	typedef enum {flush, enable, RIGHT_JOYSTICK_ARRAY_SIZE} right_joystick_buttons;

	state   m_right_joystick_current[RIGHT_JOYSTICK_ARRAY_SIZE];
	state   m_right_joystick_previous[RIGHT_JOYSTICK_ARRAY_SIZE];
	changes m_right_joystick_changes[RIGHT_JOYSTICK_ARRAY_SIZE];


public:
	Robot2012(void) 
	{
		robotDrive = new RobotDrive(driveLeftMotor, driveRightMotor);

		// Human input devices
		gamepad = 		new Gamepad(GAMEPAD);
		leftJoystick = 	new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);
		
		// Ball collector sensor switches
		switch_1 = new AnalogTrigger(ANALOG_S1);
		switch_2 = new AnalogTrigger(ANALOG_S2);
		switch_3 = new AnalogTrigger(ANALOG_S3);
		switch_4 = new AnalogTrigger(ANALOG_S4);
		
		// Drive motors and associated encoders ansd shifters
		driveLeftMotor  = 	new Jaguar(LEFT_DRIVE_MOTOR);
		driveRightMotor = 	new Jaguar(RIGHT_DRIVE_MOTOR);
		leftDriveEncoder = 	new Encoder(LEFT_DRIVE_ENCODER_A, 
										LEFT_DRIVE_ENCODER_B);
		rightDriveEncoder = new Encoder(RIGHT_DRIVE_ENCODER_A, 
										RIGHT_DRIVE_ENCODER_B);
		leftShifter = 		new Servo(LEFT_SHIFT_SERVO);
		rightShifter =		new Servo(RIGHT_SHIFT_SERVO);
		
		// Ball shooter motors, encoders, and controls
		shooterAzimuthMotor = 	new Jaguar(SHOOTER_AZIMUTH_MOTOR);
		shooterElevationMotor = new Jaguar(ELEVATION_MOTOR);
		shooterAzimuthEncoder = new Encoder(SHOOTER_AZIUMTH_ENCODER_A, 
											SHOOTER_AZIMUTH_ENCODER_B);
		shooterElevationValve = new Solenoid(SHOOTER_ELEVATION_SOLENOID);
		cameraServo = 			new Servo(CAMERA_SERVO);
		
		shooterBottomMotor =	new Jaguar(BOTTOM_SHOOTER_MOTOR);
		shooterTopMotor = 		new Jaguar(TOP_SHOOTER_MOTOR);
		bottomShooterEncoder = 	new Encoder(BOTTOM_SHOOTER_ENCODER_A,
											BOTTOM_SHOOTER_ENCODER_A);
		topShooterEncoder = 	new Encoder(TOP_SHOOTER_ENCODER_A,
											TOP_SHOOTER_ENCODER_A);
		shooterElevationValve =	new Solenoid (SHOOTER_ELEVATION_SOLENOID);
		
		// Driver station I/O
		ds = 	DriverStation::GetInstance();
		dsLCD = DriverStationLCD::GetInstance();
		dds = 	new DashboardDataFormat();
		
		// Miscellaneous
		compressor = 		new Compressor(PNEUMATIC_PRESSURE_SWITCH, COMPRESSOR);
		camera_light = 		new DigitalOutput(CAMERA_LIGHT_ENABLE);
		ultrasonicSensor =  new Ultrasonic(ULTRASONIC_ENABLE, 
											ULTRASONIC_INPUT);
		ballDisplay_0 = 	new Relay (BALL_DISPLAY_0);
		ballDisplay_1 = 	new Relay (BALL_DISPLAY_1);
		
		// Ball collector motors
		ballCollectorM1 = new Relay(BALL_COLLECTOR_M1);
		ballCollectorM2 = new Relay(BALL_COLLECTOR_M2);
		ballCollectorM3 = new Relay(BALL_COLLECTOR_M3);
		
		ballCollectorTimer = new Timer();
		armMotor = 			 new Jaguar(ARM_MOTOR);
		
		m_shooterBottomScaleFactor = 1;
		m_shooterTopScaleFactor = 1;
		m_bottomShooterMotorSetting = 0.0;
		m_topShooterMotorSetting = 0.0;
		m_bottomShooterDesiredRPM = 0;
		m_topShooterDesiredRPM = 0;	
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
		
	//TODO: Decide on the correct button to use 
	void HandleArm(void)
	{
		static bool arm_up = off;
		if (arm_up && EitherJoystickButtonEvent(7))
		{
			arm_up = true;
			armMotor->Set(Relay::kForward);
		}
		if (!arm_up && EitherJoystickButtonEvent(8))
		{
			arm_up = false;
			armMotor->Set(Relay::kReverse);
		}
	}
	
	// TODO: Get rid of numeric constants
	void HandleDriverInputs(void)
	{
		Joystick *currentJoystick = m_ds->GetDigitalIn(DS_LEFT_OR_RIGHT_STICK) ? 
												rightJoystick : leftJoystick;
		
		if (EitherJoystickButtonEvent(3))
		{
			leftShifter->SetAngle(40);
			rightShifter->SetAngle(40);
		}
		if (EitherJoystickButtonEvent(2))
		{
			leftShifter->SetAngle(140);
			rightShifter->SetAngle(140);
		}
		
		//TODO: Does this work?
		if (!m_ds->GetDigitalIn(DS_DRIVE_TYPE))
		{
			robotDrive->ArcadeDrive(currentJoystick);
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

	void TestBallCollector (void)
	{
		// Motor 1
		if (pressed == LeftJoystickButtonEvent(6))
		{
			ballCollectorM1->Set(Relay::kForward);
		}
		if (released == LeftJoystickButtonEvent(6))
		{
			ballCollectorM1->Set(Relay::kOff);
		}
		
		if (pressed == LeftJoystickButtonEvent(7))
		{
			ballCollectorM1->Set(Relay::kReverse);
		}
		if (released == LeftJoystickButtonEvent(7))
		{
			ballCollectorM1->Set(Relay::kOff);
		}
		
		if (pressed == LeftJoystickButtonEvent(8))
		{
			ballCollectorM2->Set(Relay::kForward);
		}
		if (released == LeftJoystickButtonEvent(8))
		{
			ballCollectorM2->Set(Relay::kOff);
		}
		
		// Motor 2
		if (pressed == LeftJoystickButtonEvent(9))
		{
			ballCollectorM2->Set(Relay::kReverse);
		}
		if (released == LeftJoystickButtonEvent(9))
		{
			ballCollectorM2->Set(Relay::kOff);
		}
		if (pressed == LeftJoystickButtonEvent(8))
		{
			ballCollectorM2->Set(Relay::kForward);
		}
		if (released == LeftJoystickButtonEvent(8))
		{
			ballCollectorM2->Set(Relay::kOff);
		}
		
		// Motor 3
		if (pressed == LeftJoystickButtonEvent(11))
		{
			ballCollectorM3->Set(Relay::kReverse);
		}
		if (released == LeftJoystickButtonEvent(11))
		{
			ballCollectorM3->Set(Relay::kOff);
		}
		if (pressed == LeftJoystickButtonEvent(10))
		{
			ballCollectorM3->Set(Relay::kForward);
		}
		if (released == LeftJoystickButtonEvent(10))
		{
			ballCollectorM3->Set(Relay::kOff);
		}

	}
	void DisplayCollectedBallCount(void)
	{
		
	}
	
	void UpdateDriverStation(void)
	{
		
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
			
			// Display the number of balls we are carrying on an display
			// on the outside of the robot
			DisplayCollectedBallCount();
			
			// Gather up all the data to be sent to the driver station
			// and update the driver station LCD
			UpdateDriverStation();
		}
	}
};
