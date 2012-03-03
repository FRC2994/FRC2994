#include "WPILib.h"
#include "Gamepad.h"
#include "DashboardDataFormat.h"
#include "MBUltrasonic.h"

// Access macros
#define CollectorSwitchState(i) m_collector_current[(i)]
#define CollectorSwitchEvent(i) m_collector_changes[(i)]

#define GamepadButtonState(i) m_gamepad_current[(i)]
#define GamepadButtonEvent(i) m_gamepad_changes[(i)]
#define LeftJoystickButtonState(i) m_left_joystick_current[(i)]
#define LeftJoystickButtonEvent(i) m_left_joystick_changes[(i)]
#define RightJoystickButtonState(i) m_right_joystick_current[(i)]
#define RightJoystickButtonEvent(i) m_right_joystick_changes[(i)]

// Driver station digital inputs
#define DS_DRIVE_TYPE				1
#define DS_LEFT_OR_RIGHT_STICK		2
#define DS_USE_MANUAL_DISTANCE		3
#define DS_USE_ULTRASONIC_DISTANCE	4
#define DS_USE_VISION_DISTANCE		5
#define DS_USE_KINECT				7
#define DS_DEBUG					8

// Driver station analog inputs
#define POSITION_SLIDER 		1
#define BASKET_HEIGHT_SLIDER 	2
#define DELAY_SLIDER 			3
#define BRIDGE_SLIDER 			4
// cRIO Module Assignements

// Analog Module
#define ANALOG_S1 1
#define ANALOG_S2 2
#define ANALOG_S3 3
#define ANALOG_S4 4

// cRIO Digital IOs
#define LEFT_DRIVE_ENCODER_A		1
#define LEFT_DRIVE_ENCODER_B 		2
#define RIGHT_DRIVE_ENCODER_A		3
#define RIGHT_DRIVE_ENCODER_B		4
#define BOTTOM_SHOOTER_ENCODER_A	5
#define BOTTOM_SHOOTER_ENCODER_B	6
#define TOP_SHOOTER_ENCODER_A		7
#define TOP_SHOOTER_ENCODER_B		8
#define ULTRASONIC_PING				11
#define ULTRASONIC_RX				12
#define CAMERA_LIGHT_ENABLE			13
#define PNEUMATIC_PRESSURE_SWITCH	14

// PWM
#define LEFT_DRIVE_MOTOR		1
#define RIGHT_DRIVE_MOTOR		2
#define ARM_MOTOR				3
#define BOTTOM_SHOOTER_MOTOR	5
#define TOP_SHOOTER_MOTOR		6
#define ELEVATION_MOTOR			7
#define LEFT_SHIFT_SERVO		8
#define RIGHT_SHIFT_SERVO		9
#define CAMERA_SERVO			10

// Relays
#define BALL_COLLECTOR_FRONT	1
#define BALL_COLLECTOR_MIDDLE	2
#define BALL_COLLECTOR_VERTICAL	3
#define COMPRESSOR				4
#define BALL_DISPLAY_0			5
#define BALL_DISPLAY_1			6
#define SHOOTER_FEED			7

// Solenoids

// Gamepad (real) button assignments
#define SHOOT 		1
#define LOW			2
#define MEDIUM		3
#define HIGH		4
#define INCR_SMALL 	5
#define INCR_BIG	6
#define DECR_SMALL	7
#define DECR_BIG	8
#define ENABLE		9
#define FLUSH		10

// Left joystick (real) button assignments
#define SHIFTER_BUTTON 		1	// applies to both joysticks
#define ARM_DOWN			2	// applies to both joysticks
#define ARM_UP				3	// applies to both joysticks
#define BC_FRONT_FWD		6
#define BC_FRONT_REV		7
#define BC_MIDDLE_FWD		8
#define BC_MIDDLE_REV		9
#define BC_VERTICAL_FWD		11
#define BC_VERTICAL_REV		10

// Right joystick (real) button assignments
#define SHOOTER_TOP_INCREASE	6
#define SHOOTER_TOP_DECREASE	7
#define SHOOTER_RUN_MOTORS		8
#define SHOOTER_BOTTOM_DECREASE 10
#define SHOOTER_BOTTOM_INCREASE 11

// Manual distance settings
#define SHORT_DISTANCE 		10
#define MEDIUM_DISTANCE 	20
#define LONG_DISTANCE 		30

// Analog (real) switch assignments
#define S1 1
#define S2 2
#define S3 3
#define S4 4

// Super shifter gear.
#define SHIFTER_HIGH_GEAR 40
#define SHIFTER_LOW_GEAR 140

// Human input devices
#define RIGHT_JOYSTICK	1
#define LEFT_JOYSTICK	2
#define GAMEPAD			3

// Autonomous
#define BALL_WAIT		2.0
#define ARM_WAIT		1.0

// Distance measuring
#define MANUAL_DIST_SMALL_INCREMENT	1
#define MANUAL_DIST_BIG_INCREMENT	5

// Shaft Encoder distance/pulse
// 8 inch wheel: PI*8/360 = 3.14159265*8/360 = .06981317 inches per pulse
#define DRIVE_ENCODER_DISTANCE_PER_PULSE 	0.06981317
#define SHOOTER_ENCODER_DISTANCE_PER_PULSE 	0.0027777777
// Types used for monitoring buttons and switches
typedef enum {none, released, pressed} changes;
typedef enum {off, on} state;

// Ball collector motors
typedef enum {motor_off, motor_fwd, motor_rev, NUM_BALL_COLLECTOR_MOTOR_STATES} motor_states;
const char *motorStateStrings[] = {"O", "F", "R"};
typedef enum {m1, m2, m3, m4, NUM_BALL_COLLECTOR_MOTORS} motors;

// Ball colllector states
typedef enum {I, C, D, S, W, F} collector_modes;
char *collectorModeLetters[] = {"I", "C", "D", "S", "W", "F"};

typedef enum {start_one, start_two, start_three, NUM_START_POSITION} start_positions;
typedef enum {basket_low, basket_medium, basket_high, SHOOTER_HEIGHT_ARRAY_SIZE} basket_height;
typedef enum {manual, automatic, NUM_SHOOTER_MODES} shooter_modes;


// Structure for shooter tables
#define SHOOTER_TABLE_ENTRIES 5  // this will have to be larger
typedef struct {
	float  distance;
	float  bottomMotorSetting;
	float  topMotorSetting;
	UINT32 bottomDesriedRPM;
	UINT32 topDesiredRPM;
} shooter_table;
typedef struct
{
	float speed_top;
	float speed_bottom;
} shooter_speed;

// Make sure that the start_positions enum is declared previous to these tables.
typedef struct
{
	float speed_left;
	float speed_right;
	float distance;
} step_speed;

const shooter_table m_lowerBasketTable[SHOOTER_TABLE_ENTRIES] =
		{{5.0, 0.45, 0.45, 0, 0},  // measured
		 {10.0, 0.45, 0.45, 0, 0},
		 {15.0, 0.45, 0.45, 0, 0},
		 {20.0, 0.45, 0.45, 0, 0},
		 {25.0, 0.45, 0.45, 0, 0},
		};
const shooter_table 	m_middleBasketTable[SHOOTER_TABLE_ENTRIES] =
		{{5.0, 0.55, 0.55, 0, 0},  // measured
		 {10.0, 0.60, 0.60, 0, 0}, // measured
		 {15.0, 0.70, 0.70, 0, 0}, 
		 {20.0, 0.80, 0.80, 0, 0}, // measured
		 {25.0, 0.90, 0.90, 0, 0},
		};

const shooter_table 	m_upperBasketTable[SHOOTER_TABLE_ENTRIES] =
		{{5.0, 0.60, 0.60, 0, 0},
		 {10.0, 0.70, 0.70, 0, 0}, // measured
		 {15.0, 0.75, 0.75, 0, 0}, // measured
		 {20.0, 0.875, 0.875, 0, 0},
		 {25.0, 1.00, 1.00, 0, 0}, // measured
		};

const shooter_speed m_autoShootTable[SHOOTER_HEIGHT_ARRAY_SIZE][NUM_START_POSITION] =
{
		{{.5,.5}, {.55,.55}, {.6,.6}},
		{{.65,.65}, {.7,.7}, {.8,.8}},
		{{.9,.9}, {.95,.95}, {1.0,1.0}},
};

const step_speed m_autoReverse[NUM_START_POSITION] =
{
		{.5,.5,240},
		{.5,.5,250},
		{.5,.5,260},
};
const step_speed m_autoTurnInitial[NUM_START_POSITION] =
{
		{.2,.2,250},
		{.2,.2,240},
		{.2,.2,230},
};
const step_speed m_autoToBridge[NUM_START_POSITION] =
{
		{.8,.8,400},
		{.8,.8,390},
		{.8,.8,380},
};
const step_speed m_autoTurnBridge[NUM_START_POSITION] =
{
		{.2,.2,220},
		{.2,.2,225},
		{.2,.2,230},
};

const DriverStationLCD::Line ballCollectorDebugLine = DriverStationLCD::kUser_Line6;
const double SHOOTER_TIMEOUT = 1.0;
#define SHOOTER_TEST_INCREMENT 0.05

// Camera servo constants
#define CAMERA_UPDATE_PERIOD 	0.1
#define CAMERA_SERVO_BIG_INCR 	10
#define CAMERA_SERVO_SMALL_INCR 5

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

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
	MBUltrasonic 	*ultrasonicSensor;
	Solenoid 	*cameraLight1;
	Solenoid 	*cameraLight2;

	// Motor Controllers
	Jaguar *driveLeftMotor;
	Jaguar *driveRightMotor;
	Jaguar *shooterBottomMotor;
	Jaguar *shooterTopMotor;
	Jaguar *shooterAzimuthMotor;
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
	Relay *shooterHelperMotor;

	// Misc
	RobotDrive 			*robotDrive;
//	DashboardDataFormat *dds;
	DriverStationLCD 	*dsLCD;
	DriverStation 		*ds;
	Timer 				*ballCollectorTimer;
	Compressor			*compressor;
	//			*shooterElevationValve;
	Timer				*cameraTimer;

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
	basket_height m_targetBasketHeight;
	shooter_modes m_shootMode;

	double 	m_distance;
	bool	m_shootDataRequested;
	bool	m_shootDataReady;
	bool	m_shootRequested;
	NetworkTable *table;

	float  	m_shooterBottomScaleFactor;
	float  	m_shooterTopScaleFactor;
	float 	m_bottomShooterMotorSetting;
	float 	m_topShooterMotorSetting;
	UINT32 	m_bottomShooterDesiredRPM;
	UINT32 	m_topShooterDesiredRPM;
	bool    m_shooterElevationUp;

	// We keep easily indexable (and extensible) arrays for only
	// those buttons and switches that we want to monitor. These are
	// updated at the start of each pass through the main loop

	// gamepad abstract arrays
	typedef enum {shoot, low, medium, high, incr_small, incr_big, decr_small,
				  decr_big, basket_top, basket_left, basket_right, basket_bottom,
				  camera, elevation, enable, flush, GAMEPAD_ARRAY_SIZE} gamepad_buttons;

	state   m_gamepad_current[GAMEPAD_ARRAY_SIZE];
	state   m_gamepad_previous[GAMEPAD_ARRAY_SIZE];
	changes m_gamepad_changes[GAMEPAD_ARRAY_SIZE];

	// left joystick abstract arrays
	typedef enum {shiftl, arm_upl, arm_downl, m1_fwd, m1_rev, m2_fwd, m2_rev, m3_fwd, m3_rev,
	LEFT_JOYSTICK_ARRAY_SIZE} left_joystick_buttons;

	state   m_left_joystick_current[LEFT_JOYSTICK_ARRAY_SIZE];
	state   m_left_joystick_previous[LEFT_JOYSTICK_ARRAY_SIZE];
	changes m_left_joystick_changes[LEFT_JOYSTICK_ARRAY_SIZE];

	// right joystick abstract arrays
	typedef enum {shiftr, arm_upr, arm_downr, shooter_top_inc, shooter_top_dec, shooter_bot_inc, 
				  shooter_bot_dec, shooter_mot, RIGHT_JOYSTICK_ARRAY_SIZE} right_joystick_buttons;

	state   m_right_joystick_current[RIGHT_JOYSTICK_ARRAY_SIZE];
	state   m_right_joystick_previous[RIGHT_JOYSTICK_ARRAY_SIZE];
	changes m_right_joystick_changes[RIGHT_JOYSTICK_ARRAY_SIZE];

	// Debug flag
	bool m_debug;

public:

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	Robot2012(void)
	{

		// Human input devices
		gamepad = 		new Gamepad(GAMEPAD);
		leftJoystick = 	new Joystick(LEFT_JOYSTICK);
		rightJoystick = new Joystick(RIGHT_JOYSTICK);

		// Ball collector sensor switches
		switch_1 = new AnalogTrigger(ANALOG_S1);
		switch_2 = new AnalogTrigger(ANALOG_S2);
		switch_3 = new AnalogTrigger(ANALOG_S3);
		switch_4 = new AnalogTrigger(ANALOG_S4);

		switch_1->SetLimitsVoltage (1,4);
		switch_2->SetLimitsVoltage (1,4);
		switch_3->SetLimitsVoltage (1,4);
		switch_4->SetLimitsVoltage (1,4);

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
		cameraServo = 			new Servo(CAMERA_SERVO);

		shooterBottomMotor =	new Jaguar(BOTTOM_SHOOTER_MOTOR);
		shooterTopMotor = 		new Jaguar(TOP_SHOOTER_MOTOR);
		shooterHelperMotor =	new Relay(SHOOTER_FEED);
		bottomShooterEncoder = 	new Encoder(BOTTOM_SHOOTER_ENCODER_A,
											BOTTOM_SHOOTER_ENCODER_B);
		topShooterEncoder = 	new Encoder(TOP_SHOOTER_ENCODER_A,
											TOP_SHOOTER_ENCODER_B);

		// Complete the setup of the encoders and start them
		leftDriveEncoder->SetDistancePerPulse(DRIVE_ENCODER_DISTANCE_PER_PULSE);
		leftDriveEncoder->SetMaxPeriod(1.0);
		leftDriveEncoder->SetReverseDirection(false);  // change to true if necessary
		leftDriveEncoder->Start();

		rightDriveEncoder->SetDistancePerPulse(DRIVE_ENCODER_DISTANCE_PER_PULSE);
		rightDriveEncoder->SetMaxPeriod(1.0);
		rightDriveEncoder->SetReverseDirection(false);  // change to true if necessary
		rightDriveEncoder->Start();

		bottomShooterEncoder->SetDistancePerPulse(SHOOTER_ENCODER_DISTANCE_PER_PULSE);
		bottomShooterEncoder->SetMaxPeriod(1.0);
		bottomShooterEncoder->SetReverseDirection(false);  // change to true if necessary
		bottomShooterEncoder->Start();

		topShooterEncoder->SetDistancePerPulse(SHOOTER_ENCODER_DISTANCE_PER_PULSE);
		topShooterEncoder->SetMaxPeriod(1.0);
		topShooterEncoder->SetReverseDirection(false);  // change to true if necessary
		topShooterEncoder->Start();

		// Solenoid(s)

		// Driver station I/O
		ds = 	DriverStation::GetInstance();
		dsLCD = DriverStationLCD::GetInstance();
//		dds = 	new DashboardDataFormat();

		// Miscellaneous
		compressor = 		new Compressor(PNEUMATIC_PRESSURE_SWITCH, COMPRESSOR);
		cameraLight1 = 		new Solenoid(1);
		cameraLight2 = 		new Solenoid(2);
		cameraTimer = 		new Timer();
 		ultrasonicSensor =  new MBUltrasonic(ULTRASONIC_PING,
											 ULTRASONIC_RX);
		ballDisplay_0 = 	new Relay (BALL_DISPLAY_0);
		ballDisplay_1 = 	new Relay (BALL_DISPLAY_1);

		// Ball collector motors
		ballCollectorM1 = new Relay(BALL_COLLECTOR_FRONT);
		ballCollectorM2 = new Relay(BALL_COLLECTOR_MIDDLE);
		ballCollectorM3 = new Relay(BALL_COLLECTOR_VERTICAL);

		ballCollectorTimer = new Timer();
		armMotor = 			 new Jaguar(ARM_MOTOR);

		m_shooterBottomScaleFactor 	= 1;
		m_shooterTopScaleFactor 	= 1;
		m_bottomShooterMotorSetting = 0.0;
		m_topShooterMotorSetting 	= 0.0;
		m_bottomShooterDesiredRPM 	= 0;
		m_topShooterDesiredRPM 		= 0;
		m_shooterElevationUp		= false;
		m_targetBasketHeight		= basket_low;

		m_shootDataRequested 	= false;
		m_shootDataReady	 	= false;
		m_shootRequested	 	= false;
		m_shootMode			 	= manual;
		table = NetworkTable::GetTable("2994_table");
		
		m_ballCount				= 0;
		m_collectorMode			= I;
		for (int i=0; i<NUM_BALL_COLLECTOR_MOTORS; i++)
		{
			m_motorState[i] = motor_off;
		}

		m_debug = false;
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "2012 " __TIME__);
		dsLCD->UpdateLCD();
		robotDrive = new RobotDrive(driveLeftMotor, driveRightMotor);
		robotDrive->SetExpiration(0.5);
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

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

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void GetGamepadButtons (void)
	{
		// Read the buttons directly
		m_gamepad_current[shoot] = 		gamepad->GetRawButton(SHOOT) ? on : off;
		m_gamepad_current[low] 	= 		gamepad->GetRawButton(LOW) ? on : off;
		m_gamepad_current[medium] = 	gamepad->GetRawButton(MEDIUM) ? on : off;
		m_gamepad_current[high] = 		gamepad->GetRawButton(HIGH) ? on : off;
		m_gamepad_current[incr_small] = gamepad->GetRawButton(INCR_SMALL) ? on : off;
		m_gamepad_current[incr_big] = 	gamepad->GetRawButton(INCR_BIG) ? on : off;
		m_gamepad_current[decr_small] = gamepad->GetRawButton(DECR_SMALL) ? on : off;
		m_gamepad_current[decr_big] = 	gamepad->GetRawButton(DECR_BIG) ? on : off;
		m_gamepad_current[enable] = 	gamepad->GetRawButton(ENABLE) ? on : off;
		m_gamepad_current[flush] = 		gamepad->GetRawButton(FLUSH) ? on : off;

		// Read stick buttons
		m_gamepad_current[camera] = 	gamepad->GetLeftPush() ? on : off;
		m_gamepad_current[elevation] = 	gamepad->GetRightPush() ? on : off;

		// Need to set these to off because the read can only set them
		for (int i=basket_top; i<=basket_bottom; i++)
		{
			m_gamepad_current[i] = off;
		}

		// Set the array entry if any one of the dpad buttons is pressed
		switch (gamepad->GetDPad())
		{
			case Gamepad::kUp:
				m_gamepad_current[basket_top] = on;
				break;
			case Gamepad::kLeft:
				m_gamepad_current[basket_left] = on;
				break;
			case Gamepad::kRight:
				m_gamepad_current[basket_right] = on;
				break;
			case Gamepad::kDown:
				m_gamepad_current[basket_bottom] = on;
				break;
			default:
				break;
		}

		ProcessChanges(m_gamepad_current,
					   m_gamepad_previous,
					   m_gamepad_changes,
					   GAMEPAD_ARRAY_SIZE);
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void GetLeftJoystickButtons (void)
	{
		m_left_joystick_current[shiftl]  = 		leftJoystick->GetRawButton(SHIFTER_BUTTON) ? on : off;
		m_left_joystick_current[arm_upl]  = 	leftJoystick->GetRawButton(ARM_UP) ? on : off;
		m_left_joystick_current[arm_downl]  = 	leftJoystick->GetRawButton(ARM_DOWN) ? on : off;
		m_left_joystick_current[m1_fwd] =		leftJoystick->GetRawButton(BC_FRONT_FWD) ? on : off;
		m_left_joystick_current[m1_rev] = 		leftJoystick->GetRawButton(BC_FRONT_REV) ? on : off;
		m_left_joystick_current[m2_fwd] = 		leftJoystick->GetRawButton(BC_MIDDLE_FWD) ? on : off;
		m_left_joystick_current[m2_rev] = 		leftJoystick->GetRawButton(BC_MIDDLE_REV) ? on : off;
		m_left_joystick_current[m3_fwd] = 		leftJoystick->GetRawButton(BC_VERTICAL_FWD) ? on : off;
		m_left_joystick_current[m3_rev] = 		leftJoystick->GetRawButton(BC_VERTICAL_REV) ? on : off;

		ProcessChanges(m_left_joystick_current,
					   m_left_joystick_previous,
					   m_left_joystick_changes,
					   LEFT_JOYSTICK_ARRAY_SIZE);
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void GetRightJoystickButtons (void)
	{
		m_right_joystick_current[shiftr]  = 		rightJoystick->GetRawButton(SHIFTER_BUTTON) ? on : off;
		m_right_joystick_current[arm_upr] = 		rightJoystick->GetRawButton(ARM_UP) ? on : off;
		m_right_joystick_current[arm_downr] = 		rightJoystick->GetRawButton(ARM_DOWN) ? on : off;
		m_right_joystick_current[shooter_top_inc] = rightJoystick->GetRawButton(SHOOTER_TOP_INCREASE) ? on : off;
		m_right_joystick_current[shooter_top_dec] = rightJoystick->GetRawButton(SHOOTER_TOP_DECREASE) ? on : off;
		m_right_joystick_current[shooter_bot_inc] = rightJoystick->GetRawButton(SHOOTER_BOTTOM_INCREASE) ? on : off;
		m_right_joystick_current[shooter_bot_dec] = rightJoystick->GetRawButton(SHOOTER_BOTTOM_DECREASE) ? on : off;
		m_right_joystick_current[shooter_mot] = 	rightJoystick->GetRawButton(SHOOTER_RUN_MOTORS) ? on : off;

		ProcessChanges(m_right_joystick_current,
					   m_right_joystick_previous,
					   m_right_joystick_changes,
					   RIGHT_JOYSTICK_ARRAY_SIZE);
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void GetCollectorSwitches(void)
	{
		// inverted because the wiring was inadvertently inverted
		m_collector_current[s1] = switch_1->GetTriggerState() ? off : on;
		m_collector_current[s2] = switch_2->GetTriggerState() ? off : on;
		m_collector_current[s3] = switch_3->GetTriggerState() ? off : on;
		m_collector_current[s4] = switch_4->GetTriggerState() ? off : on;

		ProcessChanges(m_collector_current,
					   m_collector_previous,
					   m_collector_changes,
					   COLLECTOR_SWITCH_ARRAY_SIZE);
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void HandleArm(void)
	{
		static bool armUp = true;

		if (m_debug)
		{
			if ((pressed == RightJoystickButtonEvent(arm_upr)) ||
				(pressed == LeftJoystickButtonEvent(arm_upl)))
			{
				armMotor->Set(-0.2);
			}
			if ((pressed == RightJoystickButtonEvent(arm_downr)) ||
				(pressed == LeftJoystickButtonEvent(arm_downl)))
			{
				armMotor->Set(0.2);
			}
			
			if ((released == RightJoystickButtonEvent(arm_upr)) ||
				(released == LeftJoystickButtonEvent(arm_upl)) ||
				(released == RightJoystickButtonEvent(arm_downr)) ||
				(released == LeftJoystickButtonEvent(arm_downl)))
			{
				armMotor->Set(0.0);
			}
		}
		else
		{
			if (!armUp && ((pressed == RightJoystickButtonEvent(arm_upr)) ||
						   (pressed == LeftJoystickButtonEvent(arm_upl))))
			{
				armUp = true;
				armMotor->Set(1.0);
			}
			else if (armUp && ((pressed == RightJoystickButtonEvent(arm_downr)) ||
					   	   	   (pressed == LeftJoystickButtonEvent(arm_downl))))
			{
				armUp = false;
				armMotor->Set(-0.2);
			}
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void HandleDriverInputs(void)
	{
		static bool high_gear = true;
		Joystick *currentJoystick = m_ds->GetDigitalIn(DS_LEFT_OR_RIGHT_STICK) ?
													   rightJoystick : leftJoystick;

		if ((pressed == LeftJoystickButtonEvent(shiftl)) || (pressed == RightJoystickButtonEvent(shiftr)) && high_gear)
		{
			high_gear = false;
			leftShifter->SetAngle(SHIFTER_LOW_GEAR);
			rightShifter->SetAngle(SHIFTER_LOW_GEAR);
		}
		else if ((pressed == LeftJoystickButtonEvent(shiftl)) || (pressed == RightJoystickButtonEvent(shiftr)) && !high_gear)
		{
			high_gear = true;
			leftShifter->SetAngle(SHIFTER_HIGH_GEAR);
			rightShifter->SetAngle(SHIFTER_HIGH_GEAR);
		}

		if (!m_ds->GetDigitalIn(DS_DRIVE_TYPE))
		{
			robotDrive->ArcadeDrive(currentJoystick);
		}
		else
		{
			robotDrive->TankDrive(leftJoystick, rightJoystick);
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void HandleCameraServo ()
	{
		// initially set to middle of range
		static float cameraAngle = Servo::GetMinAngle() + 
								   ((Servo::GetMaxAngle() - Servo::GetMinAngle()) / 2.0); 
		static float joystickPosition = 0;

		// Handle Camera Joystick
		if (cameraTimer->HasPeriodPassed(CAMERA_UPDATE_PERIOD))
		{
			// Handle camera position reset
			if (pressed == GamepadButtonEvent(camera))
			{
				cameraAngle = Servo::GetMinAngle() + 
							  ((Servo::GetMaxAngle() - Servo::GetMinAngle()) / 2.0); 
			}

			joystickPosition = gamepad->GetRightY();

			if (joystickPosition > 0.5)
			{
				cameraAngle += CAMERA_SERVO_BIG_INCR;
			}
			else if (joystickPosition > 0.1)
			{
				cameraAngle += CAMERA_SERVO_SMALL_INCR;
			}
			else if (joystickPosition < -0.5)
			{
				cameraAngle -= CAMERA_SERVO_BIG_INCR;
			}
			else if (joystickPosition < -0.1)
			{
				cameraAngle -= CAMERA_SERVO_SMALL_INCR;
			}
			
			// Update servo position
			if (cameraAngle < Servo::GetMinAngle())
			{
				cameraAngle = Servo::GetMinAngle();
			}
			if (cameraAngle > Servo::GetMaxAngle())
			{
				cameraAngle = Servo::GetMaxAngle();
			}

			cameraServo->SetAngle(cameraAngle);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "%d", cameraServo->GetAngle());
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void MakeAutoDistanceRequest(basket_height basket)
	{
		if (!m_shootDataRequested)
		{
			m_shootDataRequested = true;
			m_targetBasketHeight = basket;
			m_shootMode 		 = automatic;

			if (ds->GetDigitalIn(DS_USE_ULTRASONIC_DISTANCE) &&
				!ds->GetDigitalIn(DS_USE_VISION_DISTANCE))
			{
				m_distance = ultrasonicSensor->GetRangeInches();  // toss away the result
			}
			else if (ds->GetDigitalIn(DS_USE_VISION_DISTANCE))
			{
				//cameraLight->Set(true);
				table->PutBoolean("getDistance", true);
			}
			else
			{
				// Do nothing
			}
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void HandleShooterInputs(void)
	{
		// Handle the turret joystick
		shooterAzimuthMotor->Set(gamepad->GetLeftY());
		
		// Handle buttons 1 through 4 (manual mode distance base and shoot
		// button)
		if (pressed == GamepadButtonEvent(low))
		{
			//m_targetBasketHeight = basket_low;
			m_distance = SHORT_DISTANCE;
			m_shootDataReady = true;
			m_shootMode = manual;
		}

		if (pressed == GamepadButtonEvent(medium))
		{
			//m_targetBasketHeight = basket_medium;
			m_distance = MEDIUM_DISTANCE;
			m_shootDataReady = true;
			m_shootMode = manual;
		}

		if (pressed == GamepadButtonEvent(high))
		{
			//m_targetBasketHeight = basket_high;
			m_distance = LONG_DISTANCE;
			m_shootDataReady = true;
			m_shootMode = manual;
		}

		if (pressed == GamepadButtonEvent(shoot))
		{
			m_shootRequested = true;
		}

		// Handle buttons 5 through 8 (distance increment/decrement buttons)
		if (pressed == GamepadButtonEvent(decr_small))
		{
			m_distance -= MANUAL_DIST_SMALL_INCREMENT;
		}

		if (pressed == GamepadButtonEvent(incr_small))
		{
			m_distance += MANUAL_DIST_SMALL_INCREMENT;
		}

		if (pressed == GamepadButtonEvent(decr_big))
		{
			m_distance -= MANUAL_DIST_BIG_INCREMENT;
		}

		if (pressed == GamepadButtonEvent(incr_big))
		{
			m_distance += MANUAL_DIST_BIG_INCREMENT;
		}

		// Handle the DPad buttons (automatic mode basket select)
		if (pressed == GamepadButtonEvent(basket_top))
		{
			m_targetBasketHeight = basket_high; // for manual mode
			//MakeAutoDistanceRequest(basket_high);
		}

		if (pressed == GamepadButtonEvent(basket_left))
		{
			m_targetBasketHeight = basket_medium; // for manual mode
			//MakeAutoDistanceRequest(basket_medium);
		}

		if (pressed == GamepadButtonEvent(basket_right))
		{
			m_targetBasketHeight = basket_medium; // for manual mode
			//MakeAutoDistanceRequest(basket_medium);
		}

		if (pressed == GamepadButtonEvent(basket_bottom))
		{
			m_targetBasketHeight = basket_low; // for manual mode
			//MakeAutoDistanceRequest(basket_low);
		}
		
		if (!m_shooterElevationUp && (pressed == GamepadButtonEvent(elevation)))
		{
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2,"Setting up");
			m_shooterElevationUp = true;
			//shooterElevationValve->Set(m_shooterElevationUp);
		}
		else if ((pressed == GamepadButtonEvent(elevation)) && m_shooterElevationUp)
		{
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2,"Setting down");

			m_shooterElevationUp = false;
			//shooterElevationValve->Set(m_shooterElevationUp);
		}

		// Handle outstanding vision/ultrasonic distance requests
		if (m_shootDataRequested)
		{
			 if (ds->GetDigitalIn(DS_USE_VISION_DISTANCE))
			{
				// We have to poll the vision code on the driver station to get back the
				// distance info
				bool dataReady = table->GetBoolean("haveDistance");
				if (dataReady) {
					m_distance = table->GetDouble("distance");
					m_shootDataReady = true;
					m_shootDataRequested = false;
					//cameraLight->Set(false);
				}
			}
			else
			{
				// should not be able to get here but...
			}
		}

		// OK, now we have an updated distance (potentially anyway). Let's do the
		// lookup to get the shooter motor settings

		const shooter_table *basketTable_p = NULL;
		float lower = 0.0;

		switch (m_targetBasketHeight)
		{
			case basket_low:
				basketTable_p = m_lowerBasketTable;
				break;
			case basket_medium:
				basketTable_p = m_middleBasketTable;
				break;
			case basket_high:
				basketTable_p = m_upperBasketTable;
				break;
			default:
				break;
		}

		// Interpolate the shooter settings from the selected table
		float maxDistance = basketTable_p[SHOOTER_TABLE_ENTRIES-1].distance;

		if (m_distance > maxDistance)
		{
			m_distance = maxDistance;
		}
		else if (m_distance < 0)
		{
			m_distance = 0;
		}

		for (int i=0; i<SHOOTER_TABLE_ENTRIES; i++)
		{
			if ((m_distance >= lower) && (m_distance <= basketTable_p[i].distance))
			{
				// Interpolate
				float distanceDelta = basketTable_p[i].distance - lower;
				float scaleFactor = (basketTable_p[i].distance - m_distance)/distanceDelta;
				float bottomSpeedDelta;
				float topSpeedDelta;

				if (0 == i)
				{
					bottomSpeedDelta = basketTable_p[i].bottomMotorSetting;
					topSpeedDelta = basketTable_p[i].topMotorSetting;
				}
				else
				{
					bottomSpeedDelta = basketTable_p[i].bottomMotorSetting
										- basketTable_p[i-1].bottomMotorSetting;
					topSpeedDelta = basketTable_p[i].topMotorSetting
										- basketTable_p[i-1].topMotorSetting;
				}
				m_bottomShooterMotorSetting = basketTable_p[i].bottomMotorSetting - (scaleFactor * bottomSpeedDelta);
				m_topShooterMotorSetting 	= basketTable_p[i].topMotorSetting - (scaleFactor * bottomSpeedDelta);
				// Todo - RPM values
				break;
			}
			lower = basketTable_p[i].distance;
		}
		
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "%4.2f %4.2f %4.2f", m_distance, m_bottomShooterMotorSetting, m_topShooterMotorSetting);
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void TestShooterInputs ()
	{
		if (pressed == RightJoystickButtonEvent(shooter_top_inc))
		{
			m_topShooterMotorSetting += SHOOTER_TEST_INCREMENT;
		}
		if (pressed == RightJoystickButtonEvent(shooter_top_dec))
		{
			m_topShooterMotorSetting -= SHOOTER_TEST_INCREMENT;
		}
		if (pressed == RightJoystickButtonEvent(shooter_bot_inc))
		{
			m_bottomShooterMotorSetting += SHOOTER_TEST_INCREMENT;
		}
		if (pressed == RightJoystickButtonEvent(shooter_bot_dec))
		{
			m_bottomShooterMotorSetting -= SHOOTER_TEST_INCREMENT;
		}

		if (pressed == RightJoystickButtonEvent(shooter_mot))
		{
			// Turn motors ON
			shooterBottomMotor->Set(-m_bottomShooterMotorSetting);
			shooterTopMotor->Set(-m_topShooterMotorSetting);
			shooterHelperMotor->Set(Relay::kForward);
		}
		if (released == RightJoystickButtonEvent(shooter_mot))
		{
			// Turn motors OFF
			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "%4.2f %4.0f %4.2f %4.0f",
						  -m_bottomShooterMotorSetting, bottomShooterEncoder->GetRate()*60.0,
						  -m_topShooterMotorSetting, topShooterEncoder->GetRate()*60.0);
			shooterBottomMotor->Set(0.0);
			shooterTopMotor->Set(0.0);
			shooterHelperMotor->Set(Relay::kOff);
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void SetMotor(motor_states state, motors motor)
	{
		Relay::Value motor_setting;

		switch(state)
		{
			case motor_fwd:
				motor_setting = Relay::kForward;
				break;
			case motor_rev:
				motor_setting = Relay::kReverse;
				break;
			case motor_off:
				motor_setting = Relay::kOff;
				break;
			default:
				motor_setting = Relay::kOff;
				break;
		}

		m_motorState[motor] = state;
		switch (motor)
		{
			case m1:
				ballCollectorM1->Set(motor_setting);
				break;
			case m2:
				ballCollectorM2->Set(motor_setting);
				break;
			case m3:
				ballCollectorM3->Set(motor_setting);
				break;
			case m4:
				if(motor_off != state)
				{
// Check here for zero speeds on both motors? (that would be bad
// and gum up the works
					shooterBottomMotor->Set(-m_bottomShooterMotorSetting);
					shooterTopMotor->Set(-m_topShooterMotorSetting);
					shooterHelperMotor->Set(Relay::kForward);
					// Delay here to let motors spin up?
				}
				else
				{
// TODO: Measure both motor RPM's
					shooterBottomMotor->Set(0.0);
					shooterTopMotor->Set(0.0);
					shooterHelperMotor->Set(Relay::kOff);
				}
				break;
			default:
				break;
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	motor_states GetMotor (motors motor)
	{
		return m_motorState[motor];
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void PrintState (int switchNum,
					 bool initial)
	{
		if (initial)
		{
			dsLCD->Printf(ballCollectorDebugLine, 1, "%d:%s%d%s%s%s%s->",
						  switchNum,
						  collectorModeLetters[m_collectorMode],
						  m_ballCount,
						  motorStateStrings[GetMotor(m1)],
						  motorStateStrings[GetMotor(m2)],
						  motorStateStrings[GetMotor(m3)],
						  motorStateStrings[GetMotor(m4)]);
		}
		else
		{
			dsLCD->Printf(ballCollectorDebugLine, 11, "%s%d%s%s%s%s",
						  collectorModeLetters[m_collectorMode],
						  m_ballCount,
						  motorStateStrings[GetMotor(m1)],
						  motorStateStrings[GetMotor(m2)],
						  motorStateStrings[GetMotor(m3)],
						  motorStateStrings[GetMotor(m4)]);
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void RunBallCollectorStateMachine_1switch(void)
	{
		if (pressed == CollectorSwitchEvent(s4)) // switch 4
		{
			PrintState(4, true);
			if ((0 == m_ballCount) &&
				(motor_fwd == GetMotor(m1)) &&
				(motor_fwd == GetMotor(m2)) &&
				(motor_fwd == GetMotor(m3)) &&
				(motor_off == GetMotor(m4)))
			{
				// 0FFFO -> 1OOOO
				m_ballCount = 1;
				SetMotor (motor_off, m1);
				SetMotor (motor_off, m2);
				SetMotor (motor_off, m3);
			}
			PrintState(4, false);
		}
		
		if (m_shootDataReady && m_shootRequested) // switch 5
		{
			PrintState(5, true);
			if ((1 == m_ballCount) &&
				(motor_off == GetMotor(m1)) &&
				(motor_off == GetMotor(m2)) &&
				(motor_off == GetMotor(m3)) &&
				(motor_off == GetMotor(m4)))
			{
				// 1OOOO -> 0OOFF
				m_ballCount = 0;
				SetMotor (motor_fwd, m3);
				SetMotor (motor_fwd, m4);
				m_shootRequested = false;
			}
			PrintState(5, false);
		}
		
		if (true == ballCollectorTimer->HasPeriodPassed(SHOOTER_TIMEOUT)) // switch 6
		{
			PrintState(6, true);
			if ((0 == m_ballCount) &&
				(motor_off == GetMotor(m1)) &&
				(motor_off == GetMotor(m2)) &&
				(motor_fwd == GetMotor(m3)) &&
				(motor_fwd == GetMotor(m4)))
			{
				// 0OOFF -> 0FFFO
				SetMotor (motor_fwd, m1);
				SetMotor (motor_fwd, m2);
				SetMotor (motor_off, m4);
			}
			PrintState(6, false);
		}
		
		if (pressed == GamepadButtonEvent(enable)) // switch 7
		{
			PrintState(7, true);
			if ((0 == m_ballCount) &&
				(motor_fwd == GetMotor(m1)) &&
			    (motor_fwd == GetMotor(m2)) &&
			    (motor_fwd == GetMotor(m3)) &&
			    (motor_off == GetMotor(m4)))
			{
				// 0FFFO -> 0OOOO
				SetMotor (motor_off, m1);
				SetMotor (motor_off, m2);
				SetMotor (motor_off, m3);
			}
			else if ((0 == m_ballCount) &&
					 (motor_off == GetMotor(m1)) &&
					 (motor_off == GetMotor(m2)) &&
					 (motor_off == GetMotor(m3)) &&
					 (motor_off == GetMotor(m4)))
			{
				// 0OOOO -> 0FFFO
				SetMotor (motor_fwd, m1);
				SetMotor (motor_fwd, m2);
				SetMotor (motor_fwd, m3);
			}
			else if ((0 == m_ballCount) &&
					 (motor_off == GetMotor(m1)) &&
					 (motor_off == GetMotor(m2)) &&
					 (motor_fwd == GetMotor(m3)) &&
					 (motor_fwd == GetMotor(m4)))
			{
				// 0OOFF -> 0OOOO
				SetMotor (motor_off, m3);
				SetMotor (motor_off, m4);
			}
			PrintState(7, false);			
		}
		
		if (pressed == GamepadButtonEvent(flush)) // switch 8
		{
			PrintState(8, true);
			// XXXXXX -> F0RRRO
			m_ballCount = 0;
			SetMotor (motor_rev, m1);
			SetMotor (motor_rev, m2);
			SetMotor (motor_rev, m3);
			SetMotor (motor_off, m4);
			PrintState(8, false);
		}

		if(released == GamepadButtonEvent(flush))
		{
			PrintState(8, true);
			// XXXXXX -> I0OOOO
			m_ballCount = 0;
			SetMotor (motor_off, m1);
			SetMotor (motor_off, m2);
			SetMotor (motor_off, m3);
			SetMotor (motor_off, m4);
			PrintState(8, false);
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//
	
	void RunBallCollectorStateMachine_3switch(void)
	{
		if (pressed == CollectorSwitchEvent(s2)) // switch 2
		{
			PrintState(2, true);
			if ((motor_fwd == GetMotor(m1)) &&
				(motor_fwd == GetMotor(m2)) &&
				(motor_off == GetMotor(m4)))
			{
				if ((I == m_collectorMode) &&
					(0 == m_ballCount) &&
					(motor_off == GetMotor(m3)))
				{
					// I0FFOO -> C1FFFO	
					m_collectorMode = C;
					m_ballCount = 1;
					SetMotor (motor_fwd, m3);

				}
				else if ((I == m_collectorMode) &&
						(1 == m_ballCount) &&
						(motor_off == GetMotor(m3)))
				{
					// I1FFOO -> C2FFFO
					m_collectorMode = C;
					m_ballCount = 2;
					SetMotor (motor_fwd, m3);

				}
				else if ((C == m_collectorMode) &&
						(1 == m_ballCount) &&
						(motor_fwd == GetMotor(m3)))
				{	
					// C1FFFO -> C2FFFO
					m_ballCount = 2;
				}
				else if ((I == m_collectorMode) &&
						(2 == m_ballCount) &&
						(motor_off == GetMotor(m3)))
				{
					// I2FFOO -> I3OOOO
					m_ballCount = 3;
					SetMotor (motor_off, m1);
					SetMotor (motor_off, m2);
				}
				else if ((C == m_collectorMode) &&
						(2 == m_ballCount) &&
						(motor_fwd == GetMotor(m3)))
				{
					// C2FFFO -> C3OOFO
					m_ballCount = 3;
					SetMotor (motor_off, m1);
					SetMotor (motor_off, m2);
				}
			}
			PrintState(2, false);
		}
		
		if (pressed == CollectorSwitchEvent(s3)) // switch 3
		{
			PrintState(3, true);
			if ((C == m_collectorMode) &&
				(1 == m_ballCount) &&
				(motor_fwd == GetMotor(m1)) &&
				(motor_fwd == GetMotor(m2)) &&
				(motor_fwd == GetMotor(m3)) &&
				(motor_off == GetMotor(m4)))
			{
				// C1FFFO -> I1FFOO
				m_collectorMode = I;
				SetMotor (motor_off, m3);
			}
			PrintState(3, false);
		}
		
		if (pressed == CollectorSwitchEvent(s4)) // switch 4
		{
			PrintState(4, true);
			if ((motor_fwd == GetMotor(m3)))
			{
				if ((C == m_collectorMode) &&
					(2 == m_ballCount) &&
					(motor_fwd == GetMotor(m1)) &&
					(motor_fwd == GetMotor(m2)) &&
					(motor_off == GetMotor(m4)))
				{
					// C2FFFO -> I2FFOO
					m_collectorMode = I;
					SetMotor (motor_off, m3);
				}
				else if ((S == m_collectorMode) &&
					(3 == m_ballCount) &&
					(motor_off == GetMotor(m1)) &&
					(motor_fwd == GetMotor(m2)) &&
					(motor_fwd == GetMotor(m4)))
				{
					// S3OFFF -> I2FFOO
					m_collectorMode = I;
					m_ballCount = 2;
					SetMotor (motor_fwd, m1);
					SetMotor (motor_off, m3);
					SetMotor (motor_off, m4);
				}
				else if ((S == m_collectorMode) &&
					(2 == m_ballCount) &&
					(motor_off == GetMotor(m1)) &&
					(motor_off == GetMotor(m2)) &&
					(motor_fwd == GetMotor(m4)))
				{
					// S2OOFF -> I1OOOO
					m_collectorMode = I;
					m_ballCount = 1;
					SetMotor (motor_off, m3);
					SetMotor (motor_off, m4);
				}
				else if ((S == m_collectorMode) &&
					(1 == m_ballCount) &&
					(motor_off == GetMotor(m1)) &&
					(motor_off == GetMotor(m2)) &&
					(motor_off == GetMotor(m4)))
				{
					// S1OOFO -> W0OOFF
					m_collectorMode = W;
					m_ballCount = 0;
					SetMotor (motor_fwd, m4);
				}
				else if ((C == m_collectorMode) &&
					(3 == m_ballCount) &&
					(motor_off == GetMotor(m1)) &&
					(motor_off == GetMotor(m2)) &&
					(motor_off == GetMotor(m4)))
				{
					// C3OOFO -> I3OOOO
					m_collectorMode = I;
					SetMotor (motor_off, m3);
				}
			}
			PrintState(4, true);
		}
		
		if (m_shootDataReady && m_shootRequested) // switch 5
		{
			PrintState(5, true);
			if ((I == m_collectorMode) &&
				(motor_off == GetMotor(m3)) &&
				(motor_off == GetMotor(m4)))
			{
				if	((1 == m_ballCount) &&
					 (motor_fwd == GetMotor(m1)) &&
					 (motor_fwd == GetMotor(m2)))
				{
					// I1FFOO -> S1OOFO
					m_collectorMode = S;
					SetMotor (motor_off, m1);
					SetMotor (motor_off, m2);
					SetMotor (motor_fwd, m3);
					m_shootRequested = false;
				}
				else if	((2 == m_ballCount) &&
					 (motor_fwd == GetMotor(m1)) &&
					 (motor_fwd == GetMotor(m2)))
				{
					// I2FFOO -> S2OOFF
					m_collectorMode = S;
					SetMotor (motor_off, m1);
					SetMotor (motor_off, m2);
					SetMotor (motor_fwd, m3);
					SetMotor (motor_fwd, m4);
					m_shootRequested = false;
				}
				else if	((3 == m_ballCount) &&
					 (motor_off == GetMotor(m1)) &&
					 (motor_off == GetMotor(m2)))
				{
					// I3OOOO -> S3OFFF
					m_collectorMode = S;
					SetMotor (motor_fwd, m2);
					SetMotor (motor_fwd, m3);
					SetMotor (motor_fwd, m4);
					m_shootRequested = false;
				}
				else if	((1 == m_ballCount) &&
					 (motor_off == GetMotor(m1)) &&
					 (motor_off == GetMotor(m2)))
				{
					// I1OOOO -> W0OOFF
					m_collectorMode = W;
					m_ballCount = 0;
					SetMotor (motor_fwd, m3);
					SetMotor (motor_fwd, m4);
					m_shootRequested = false;
				}
			}
			PrintState(5, false);
		}
		
		if (true == ballCollectorTimer->HasPeriodPassed(SHOOTER_TIMEOUT)) // switch 6
		{
			PrintState(6, true);
			if ((W == m_collectorMode) &&
				(0 == m_ballCount) &&
				(motor_off == GetMotor(m1)) &&
				(motor_off == GetMotor(m2)) &&
				(motor_fwd == GetMotor(m3)) &&
				(motor_fwd == GetMotor(m4)))
			{
				// W0OOFF -> I0FFOO
				m_collectorMode = I;
				SetMotor (motor_fwd, m1);
				SetMotor (motor_fwd, m2);
				SetMotor (motor_off, m3);
				SetMotor (motor_off, m4);

			}
			PrintState(6, false);
		}
		
		if (pressed == GamepadButtonEvent(enable)) // switch 7
		{
			PrintState(7, true);
			if ((I == m_collectorMode) &&
				(0 == m_ballCount) &&
				(motor_off == GetMotor(m1)) &&
				(motor_off == GetMotor(m2)) &&
				(motor_off == GetMotor(m3)) &&
				(motor_off == GetMotor(m4)))
			{
				// I0OOOO -> I0FFOO
				SetMotor (motor_fwd, m1);
				SetMotor (motor_fwd, m2);
			}
			else 
			{
				// !I0OOOO -> I0OOOO
				m_collectorMode = I;
				m_ballCount = 0;
				SetMotor (motor_off, m1);
				SetMotor (motor_off, m2);
				SetMotor (motor_off, m3);
				SetMotor (motor_off, m4);
			}
			PrintState(7, false);
		}
		
		if (pressed == GamepadButtonEvent(flush)) // switch 8
		{
			PrintState(8, true);
			// XXXXXX -> F0RRRO
			m_collectorMode = F;
			m_ballCount = 0;
			SetMotor (motor_rev, m1);
			SetMotor (motor_rev, m2);
			SetMotor (motor_rev, m3);
			SetMotor (motor_off, m4);
			PrintState(8, false);
		}

		if(released == GamepadButtonEvent(flush))
		{
			PrintState(8, true);
			// XXXXXX -> I0OOOO
			m_collectorMode = I;
			m_ballCount = 0;
			SetMotor (motor_off, m1);
			SetMotor (motor_off, m2);
			SetMotor (motor_off, m3);
			SetMotor (motor_off, m4);
			PrintState(8, false);
		}
	}
	
//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//
	
	void RunBallCollectorStateMachine(void)
	{
		//dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "1:%d,2:%d,3:%d,4:%d", switch_1->GetTriggerState(), switch_2->GetTriggerState(), switch_3->GetTriggerState(), switch_4->GetTriggerState());
		dsLCD->UpdateLCD();
		if (pressed == CollectorSwitchEvent(s1)) // switch 1
		{
			PrintState(1, true);
			if ((motor_fwd == GetMotor(m1)) &&
				(motor_fwd == GetMotor(m2)) &&
				(motor_off == GetMotor(m3)) &&
				(motor_off == GetMotor(m4)))
			{
				// xxFFOO
				if (I == m_collectorMode)
				{
					switch (m_ballCount)
					{
						case 0:
							// I0FFOO -> C1FFOO
							m_collectorMode = C;
							m_ballCount++;
							break;
						case 1:
							// I1FF00 -> D2FFOO
							m_collectorMode = D;
							m_ballCount++;
							break;
						case 2:
							// 12FFOO -> C3FFOO
							m_collectorMode = C;
							m_ballCount++;
							break;
						default:
							// do nothing
							break;
					}
				}
				else if (C == m_collectorMode)
				{
					switch (m_ballCount)
					{
						case 1:
							// C1FFOO -> C2FFOO
							m_ballCount++;
							break;
						case 3:
							// C3FFOO -> C4RROO
							m_ballCount++;
							SetMotor(motor_rev, m1);
							SetMotor(motor_rev, m2);
							break;
						case 4:
							// C4RROO -> C3FFOO
							m_ballCount--;
							SetMotor(motor_fwd, m1);
							SetMotor(motor_fwd, m2);
							break;
						default:
							// do nothing
							break;
					}
				}
				else if (D == m_collectorMode)
				{
					switch (m_ballCount)
					{
						case 2:
							// D2FFOO -> D3FFOO
							m_ballCount++;
							break;
						default:
							// do nothing
							break;
					}
				}
				else
				{
					// do nothing
				}
			}
			else if ((C == m_collectorMode) &&
					(motor_fwd == GetMotor(m1)) &&
					(motor_fwd == GetMotor(m2)) &&
					(motor_off == GetMotor(m4)))
			{
				// CxFFxO
				if ((motor_fwd == GetMotor(m3)) &&
						(1 == m_ballCount))
				{
					// C1FFFO -> C2motor_offO
					m_ballCount++;
					SetMotor(motor_off, m1);
				}
				else if ((motor_off == GetMotor(m3)) &&
					 (3 == m_ballCount))
				{
					// C3FFOO -> C4RROO
					m_ballCount++;
					SetMotor(motor_rev, m1);
					SetMotor(motor_rev, m2);
				}
				else
				{
					// do nothing
				}
			}
			else if ((C == m_collectorMode) &&
					(4 == m_ballCount) &&
					(motor_rev == GetMotor(m1)) &&
					(motor_rev == GetMotor(m2)) &&
					(motor_off == GetMotor(m3)) &&
					(motor_off == GetMotor(m4)))
			{
				// C4RROO -> C3FFOO
				SetMotor(motor_fwd, m1);
				SetMotor(motor_fwd, m2);
				m_ballCount--;
			}
			else
			{
				// do nothing
			}
			PrintState(1, false);
		}

		if (pressed == CollectorSwitchEvent(s2)) // switch 2
		{
			PrintState(2, true);
				if ((motor_fwd == GetMotor(m1)) &&
					(motor_fwd == GetMotor(m2)) &&
					(motor_off == GetMotor(m3)) &&
					(motor_off == GetMotor(m4)))
				{
					if (C == m_collectorMode)
					{
						switch (m_ballCount)
						{
							case 1: // C1FFOO -> C1FFFO
								SetMotor(motor_fwd, m3);
								break;
							case 2: // C2FFOO -> C2motor_offO
								SetMotor(motor_off, m1);
								SetMotor(motor_fwd, m3);
								break;
							case 3: // C3FFOO -> I3OOOO
								SetMotor(motor_off, m1);
								SetMotor(motor_off, m2);
								m_collectorMode = I;
								break;
							default:
								//do nothing
								break;
							}
					}
					else if (D == m_collectorMode)
					{
						switch (m_ballCount)
						{
							case 2: // D2FFOO -> D2motor_offO
								SetMotor(motor_off, m1);
								SetMotor(motor_fwd, m3);
								break;
							case 3: // D3FFOO -> D3motor_offO
								SetMotor(motor_off, m1);
								SetMotor(motor_fwd, m3);
								break;
							default:
								//do nothing
								break;
						}
					}
					else
						{
							// do nothing
						}
					}
				else
				{
					// do nothing
				}
				PrintState(2, false);
		}

		if (pressed == CollectorSwitchEvent(s3)) // switch 3
		{
			PrintState(3, true);
			if ((motor_fwd == GetMotor(m2)) &&
				(motor_fwd == GetMotor(m3)) &&
				(motor_off == GetMotor(m4)))
			{
				// xxxFFO
				if (C == m_collectorMode)
				{
					if ((1 == m_ballCount) &&
						(motor_fwd == GetMotor(m1)))
					{
						// C1F
						m_collectorMode = I;
						SetMotor (motor_off, m3);
					}
					else if ((2 == m_ballCount) &&
							 (motor_off == GetMotor(m1)))
					{
						// C2O
						m_collectorMode = D;
						SetMotor (motor_fwd, m1);
						SetMotor (motor_off, m3);
					}
					else
					{
						// do nothing
					}
				}
				else if (D == m_collectorMode)
				{
					if ((3 == m_ballCount) &&
						(motor_off == GetMotor(m1)))
					{
						// D3O
						m_collectorMode = C;
						SetMotor (motor_fwd, m1);
						SetMotor (motor_off, m3);
					}
					else if ((2 == m_ballCount) &&
							 (motor_off == GetMotor(m1)))
					{
						// D2O
						m_collectorMode = I;
						SetMotor (motor_fwd, m1);
						SetMotor (motor_off, m3);
					}
					else
					{
						// do nothing
					}
				}
				else
				{
					// do nothing
				}
			}
			else
			{
				// do nothing
			}
			PrintState(3, false);
		}

		if (pressed == CollectorSwitchEvent(s4)) // switch 4
		{
			PrintState(4, true);
			if ((motor_off == GetMotor(m1)) &&
				(motor_off == GetMotor(m2)) &&
				(motor_fwd == GetMotor(m3)) &&
				(motor_fwd == GetMotor(m4)) &&
				(S == m_collectorMode))
			{
				// xxOmotor_off
				if (1 == m_ballCount)
				{
					// S1 S1Omotor_off -> W0Omotor_off
					m_collectorMode = W;
					m_ballCount--;
					ballCollectorTimer->Start();
				}
				else if (2 == m_ballCount)
				{
					// S2
					m_collectorMode = I;
					SetMotor (motor_off, m3);
					SetMotor (motor_off, m4);
					m_ballCount--;
				}
				else
				{
					// do nothing
				}
			}
			else if ((motor_off == GetMotor(m1)) &&
					 (motor_fwd == GetMotor(m2)) &&
					 (motor_fwd == GetMotor(m3)) &&
					 (motor_off == GetMotor(m4)) &&
					 (D == m_collectorMode))
			{
				// xxmotor_offO
				if (2 == m_ballCount)
				{
					// D2
					m_collectorMode = I;
					SetMotor (motor_fwd, m1);
					SetMotor (motor_off, m3);
				}
				else if (3 == m_ballCount)
				{
					// D3
					m_collectorMode = C;
					SetMotor (motor_fwd, m1);
					SetMotor (motor_off, m3);
				}
				else
				{
					// do nothing
				}
			}
			else if ((S == m_collectorMode) &&
					(3 == m_ballCount) &&
					(motor_off == GetMotor(m1)) &&
					(motor_fwd == GetMotor(m2)) &&
					(motor_fwd == GetMotor(m3)) &&					
					(motor_fwd == GetMotor(m4)))
			{
				// S3OFFF -> I2FFOO
				m_collectorMode = I;
				m_ballCount--;
				SetMotor (motor_fwd, m1);
				SetMotor (motor_off, m3);
				SetMotor (motor_off, m4);
			}
			else
			{
				// do nothing
			}
			PrintState(4, false);
		}

		if (m_shootDataReady && m_shootRequested) // switch 5
		{
			PrintState(5, true);
			if ((I == m_collectorMode) &&
				(motor_off == GetMotor(m3)) &&
				(motor_off == GetMotor(m4)))
			{
				// IxxxOO
				switch (m_ballCount)
				{
					case 1:
						if ((motor_fwd == GetMotor(m1)) &&
							(motor_fwd == GetMotor(m2)))
						{
							// 1FF
							m_collectorMode = S;
							SetMotor (motor_fwd, m4);
							SetMotor (motor_off, m1);
							SetMotor (motor_off, m2);
							SetMotor (motor_fwd, m3);
							m_shootRequested = false;
						}
						else if ((motor_off == GetMotor(m1)) &&
								 (motor_off == GetMotor(m2)))
						{
							// 1OO
							m_collectorMode = W;
							SetMotor (motor_fwd, m4);
							SetMotor (motor_fwd, m3);
							m_ballCount--;
							ballCollectorTimer->Start();
							m_shootRequested = false;
						}
						else
						{
							// do nothing
						}
						break;
					case 2:
						if ((motor_fwd == GetMotor(m1)) &&
							(motor_fwd == GetMotor(m2)))
						{
							// 2FF
							m_collectorMode = S;
							SetMotor (motor_fwd, m4);
							SetMotor (motor_off, m1);
							SetMotor (motor_off, m2);
							SetMotor (motor_fwd, m3);
							m_shootRequested = false;
						}
						else
						{
							// do nothing
						}
						break;
					case 3:
						if ((motor_off == GetMotor(m1)) &&
							(motor_off == GetMotor(m2)))
						{
							// 3OO
							m_collectorMode = S;
							SetMotor (motor_fwd, m4);
							SetMotor (motor_fwd, m2);
							SetMotor (motor_fwd, m3);
							m_shootRequested = false;
						}
						else
						{
							// do nothing
						}
						break;
					default:
						break;
				}
			}
			else
			{
				// do nothing
			}
			PrintState(5, false);
		}

		if (true == ballCollectorTimer->HasPeriodPassed(SHOOTER_TIMEOUT)) // switch 6
		{
			PrintState(6, true);
			if ((W == m_collectorMode) &&
				(0 == m_ballCount) &&
				(motor_off == GetMotor(m1)) &&
				(motor_off == GetMotor(m2)) &&
				(motor_fwd == GetMotor(m3)) &&
				(motor_fwd == GetMotor(m4)))
			{
				// W0OOFF -> I0FFOO
				m_collectorMode = I;
				SetMotor (motor_fwd, m1);
				SetMotor (motor_fwd, m2);
				SetMotor (motor_off, m3);
				SetMotor (motor_off, m4);
// TODO: read shooter motor RPMs, compare to desired values and recompute a scaling factor
// to use in the next shot
				ballCollectorTimer->Stop();
			}
			else
			{
				// do nothing
			}
			PrintState(6, false);
		}

		if (pressed == GamepadButtonEvent(enable)) // switch 7
		{
			PrintState(7, true);
			if ((I == m_collectorMode) &&
				(0 == m_ballCount) &&
				(motor_off == GetMotor(m1)) &&
				(motor_off == GetMotor(m2)) &&
				(motor_off == GetMotor(m3)) &&
				(motor_off == GetMotor(m4)))
			{
				// I0OOOO -> I0FFOO
				m_collectorMode = I;
				SetMotor (motor_fwd, m1);
				SetMotor (motor_fwd, m2);
				SetMotor (motor_off, m3);
				SetMotor (motor_off, m4);
			}
			else
			{
				// XXXXXX -> I0OOOO
				m_collectorMode = I;
				m_ballCount = 0;
				SetMotor (motor_off, m1);
				SetMotor (motor_off, m2);
				SetMotor (motor_off, m3);
				SetMotor (motor_off, m4);
			}
			PrintState(7, false);
		}

		if(pressed == GamepadButtonEvent(flush)) // switch 8
		{
			PrintState(8, true);
			// XXXXXX -> F0RRRO
			m_collectorMode = F;
			m_ballCount = 0;
			SetMotor (motor_rev, m1);
			SetMotor (motor_rev, m2);
			SetMotor (motor_rev, m3);
			SetMotor (motor_off, m4);
			PrintState(8, false);
		}

		if(released == GamepadButtonEvent(flush))
		{
			PrintState(8, true);
			// XXXXXX -> I0OOOO
			m_collectorMode = I;
			m_ballCount = 0;
			SetMotor (motor_off, m1);
			SetMotor (motor_off, m2);
			SetMotor (motor_off, m3);
			SetMotor (motor_off, m4);
			PrintState(8, false);
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void TestBallCollector (void)
	{
		// Motor 1
		if (pressed == LeftJoystickButtonEvent(m1_fwd))
		{
			ballCollectorM1->Set(Relay::kForward);
		}
		if (released == LeftJoystickButtonEvent(m1_fwd))
		{
			ballCollectorM1->Set(Relay::kOff);
		}

		if (pressed == LeftJoystickButtonEvent(m1_rev))
		{
			ballCollectorM1->Set(Relay::kReverse);
		}
		if (released == LeftJoystickButtonEvent(m1_rev))
		{
			ballCollectorM1->Set(Relay::kOff);
		}

		// Motor 2
		if (pressed == LeftJoystickButtonEvent(m2_rev))
		{
			ballCollectorM2->Set(Relay::kReverse);
		}
		if (released == LeftJoystickButtonEvent(m2_rev))
		{
			ballCollectorM2->Set(Relay::kOff);
		}
		if (pressed == LeftJoystickButtonEvent(m2_fwd))
		{
			ballCollectorM2->Set(Relay::kForward);
		}
		if (released == LeftJoystickButtonEvent(m2_fwd))
		{
			ballCollectorM2->Set(Relay::kOff);
		}

		// Motor 3
		if (pressed == LeftJoystickButtonEvent(m3_rev))
		{
			ballCollectorM3->Set(Relay::kReverse);
		}
		if (released == LeftJoystickButtonEvent(m3_rev))
		{
			ballCollectorM3->Set(Relay::kOff);
		}
		if (pressed == LeftJoystickButtonEvent(m3_fwd))
		{
			ballCollectorM3->Set(Relay::kForward);
		}
		if (released == LeftJoystickButtonEvent(m3_fwd))
		{
			ballCollectorM3->Set(Relay::kOff);
		}
	}

//----------------------------------------------------------------------------//
//
// Show the world our ball count on the external display
//
//----------------------------------------------------------------------------//

	void DisplayCollectedBallCount(void)
	{
		int count = m_ballCount;

		if (m_ballCount > 3)
		{
			m_ballCount = 3;
		}

		ballDisplay_0->Set((count & 1) ? Relay::kOn : Relay::kOff);
		ballDisplay_1->Set(((count >> 1) & 1) ? Relay::kOn : Relay::kOff);
	}
	
//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void UpdateDriverStation(void)
	{
		// Put any general info here
		dsLCD->UpdateLCD();
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void TestBounce (void)
	{
		static int pressedCount = 0;
		static int releasedCount = 0;
		
		// Look for pressed event and increment count
		if(pressed == CollectorSwitchEvent(s4)) // switch
		{
			pressedCount++;
		}
		// Look for released event and increment count
		if(released == CollectorSwitchEvent(s4)) // switch
		{
			releasedCount++;
		}
		
		// Read the switch state and display it plus the counts
		dsLCD->Printf(ballCollectorDebugLine, 1, "st: %d pr: %d re: %d",
				CollectorSwitchState(m4), pressedCount, releasedCount);
	}
	
//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void TmpOperatorControl(void)
	{
		robotDrive->SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			robotDrive->ArcadeDrive(leftJoystick); // drive with arcade style (use right stick)
			Wait(0.005);				// wait for a motor update time
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void OperatorControl(void)
	{
		leftShifter->SetAngle(40);
		leftShifter->SetAngle(40);
		robotDrive->SetSafetyEnabled(true);
		// Call once to initialize current states. First call in loop should get
		// same current states thus setting all previous equal to current and off
		// we go...
		GetGamepadButtons();
		GetLeftJoystickButtons ();
		GetRightJoystickButtons ();
		GetCollectorSwitches();

		// Start the compressor
		compressor->Start();

		// Start the timer that controls how fast the camera servo can be updated
		cameraTimer->Start();

		// Set the elevation to "down"
		m_shooterElevationUp = false;
		//shooterElevationValve->Set(m_shooterElevationUp);

		//----------------------------------------------------------------------------//
		// Main loop
		//----------------------------------------------------------------------------//

		while (IsOperatorControl())
		{
			cameraLight1->Set(true);
			cameraLight2->Set(true);
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "l:%d r:%d", leftShifter->GetAngle(), rightShifter->GetAngle());
			// Set debug flag
			m_debug = m_ds->GetDigitalIn(DS_DEBUG);

			// Get inputs that we need to know both current and previous state
			GetGamepadButtons();
			GetLeftJoystickButtons ();
			GetRightJoystickButtons ();
			GetCollectorSwitches();

			// Make any necessary changes to the arm
			HandleArm ();

			// Drive the robot
			HandleDriverInputs();

			// Handle camera position change requests
			HandleCameraServo();

			// Process the shooter button and joystick inputs.
			if(m_debug)
			{
				TestBallCollector();
				TestShooterInputs();
			}
			else
			{
				RunBallCollectorStateMachine();
				HandleShooterInputs();
			}

			// Handle the collection of balls from the floor automatically
			if(m_debug)
			{
				TestBallCollector();
			}
			else
			{
				RunBallCollectorStateMachine();
				//RunBallCollectorStateMachine_1switch();
				//RunBallCollectorStateMachine_3switch();
			}

			// Display the number of balls we are carrying on an display
			// on the outside of the robot
			//DisplayCollectedBallCount();
			
			// Gather up all the data to be sent to the driver station
			// and update the driver station LCD
			UpdateDriverStation();
		}
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void ClearDSLine(DriverStationLCD::Line line)
	{
		dsLCD->PrintfLine(line, "                    ");
		dsLCD->UpdateLCD();
	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void DoAutonomousMoveStep(const step_speed *speeds, const start_positions initial, char * message)
	{
		leftDriveEncoder->Reset();

		ClearDSLine(DriverStationLCD::kUser_Line2);
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "i: %d %s", initial, message);
		dsLCD->UpdateLCD();
		driveLeftMotor->Set(speeds[initial].speed_left);
		driveRightMotor->Set(speeds[initial].speed_right);
		double dist = speeds[initial].distance;
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "d:%5.2f e:%5.2f", dist, leftDriveEncoder->GetDistance());
		dsLCD->UpdateLCD();

		while (dist > leftDriveEncoder->GetDistance())
		{
			dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "d:%5.2f e:%5.2f", dist, leftDriveEncoder->GetDistance());
			dsLCD->UpdateLCD();
			Wait(0.01);
		}
		driveLeftMotor->Set(0.0);
		driveRightMotor->Set(0.0);

	}

//----------------------------------------------------------------------------//
//
//----------------------------------------------------------------------------//

	void Autonomous(void)
	{
		float positionSlider = 0.0;
		float basketHeightSlider = 0.0;
		float shootDelay = 0.0;
		start_positions position;
		basket_height   which_basket;
		// TODO: Shifter default value? (Supposed to be low-gear. Maybe we should make it low for the more "percision" parts of autonomous, but for others make it high).

		robotDrive->SetSafetyEnabled(false);
		// Read configuration from driver station
		// - position (analog slider 1)
		positionSlider = ds->GetAnalogIn(POSITION_SLIDER);
		if (positionSlider < 1.0)
		{
			position = start_one;
		}
		else if (positionSlider < 2.0)
		{
			position = start_two;
		}
		else
		{
			position = start_three;
		}

		// - basket level to target (analog slider 2)
		basketHeightSlider = ds->GetAnalogIn(BASKET_HEIGHT_SLIDER);
		if (basketHeightSlider < 1.0)
		{
			which_basket = basket_low;
		}
		else if (basketHeightSlider < 2.0)
		{
			which_basket = basket_medium;
		}
		else
		{
			which_basket = basket_high;
		}

		// - delay until shooting first basket (analog slider 3)
		shootDelay = ds->GetAnalogIn(DELAY_SLIDER);


		dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "Shooting...");
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "p: %d b: %d d:%5.2f", position, which_basket, shootDelay);
		
		dsLCD->UpdateLCD();

		// Shoot two balls
		Wait(shootDelay);
		dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "bot: %5.2f",m_autoShootTable[which_basket][position].speed_bottom);
		shooterBottomMotor->Set(m_autoShootTable[which_basket][position].speed_bottom);
		shooterTopMotor->Set(m_autoShootTable[which_basket][position].speed_top);
		shooterHelperMotor->Set(Relay::kForward);

		Wait(BALL_WAIT);

		shooterBottomMotor->Set(0);
		shooterTopMotor->Set(0);
		shooterHelperMotor->Set(Relay::kOff);

		// Go to bridge

		//TODO: Make this a constant
		DoAutonomousMoveStep(m_autoReverse, position, "Reversing...");

		DoAutonomousMoveStep(m_autoTurnInitial, position, "Doing initial turn...");

		DoAutonomousMoveStep(m_autoToBridge, position, "Driving to bridge...");

		DoAutonomousMoveStep(m_autoTurnBridge, position, "Turning to face bridge...");

		// Tip bridge to release balls
		armMotor->Set(-1.0);
		Wait(ARM_WAIT);
		armMotor->Set(1.0);

		// - are we in kinect mode? (digital IO ??)

	}
};

START_ROBOT_CLASS(Robot2012);
