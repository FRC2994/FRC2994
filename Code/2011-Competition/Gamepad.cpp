
#include "Gamepad.h"
#include "DriverStation.h"
#include "Utility.h"
#include "WPIStatus.h"

/**
 * Construct an instance of a Gamepad.
 *
 * @param port The port on the driver station that the gamepad is plugged into.
 */
Gamepad::Gamepad(UINT32 port) : Joystick(port)
{
    a_port = port;
    ap_ds = DriverStation::GetInstance();
}

Gamepad::~Gamepad()
{
}

/**
 * Get the X value of the left analog stick.
 */
float Gamepad::GetLeftX()
{
    return GetRawAxis(kLeftXAxisNum);
}

/**
 * Get the Y value of the left analog stick.
 */
float Gamepad::GetLeftY()
{
    return GetRawAxis(kLeftYAxisNum);
}

/**
 * Get the X value of the right analog stick.
 */
float Gamepad::GetRightX()
{
    return GetRawAxis(kRightXAxisNum);
}

/**
 * Get the Y value of the right analog stick.
 */
float Gamepad::GetRightY()
{
    return GetRawAxis(kRightYAxisNum);
}

/**
 * Get the value of the axis.
 *
 * @param axis The axis to read [1-6].
 * @return The value of the axis.
 */
float Gamepad::GetRawAxis(UINT32 axis)
{
    return ap_ds->GetStickAxis(a_port, axis);
}

/**
 * Return the axis determined by the argument.
 *
 * This is for cases where the gamepad axis is returned programatically,
 * otherwise one of the previous functions would be preferable (for example
 * GetLeftX()).
 *
 * @param axis The axis to read.
 * @return The value of the axis.
 */
float Gamepad::GetAxis(AxisType axis)
{
    switch(axis)
    {
        case kLeftXAxis: return GetLeftX();
        case kLeftYAxis: return GetLeftY();
        case kRightXAxis: return GetRightX();
        case kRightYAxis: return GetRightY();
        default:
            wpi_fatal(BadJoystickAxis);
            return 0.0;
    }
}

/**
 * Get the button value for buttons 1 through 12.
 *
 * The buttons are returned in a single 16 bit value with one bit representing
 * the state of each button. The appropriate button is returned as a boolean
 * value.
 *
 * @param button The button number to be read.
 * @return The state of the button.
 **/
bool Gamepad::GetNumberedButton(UINT32 button)
{
    return ((0x1 << (button-1)) & ap_ds->GetStickButtons(a_port)) != 0;
}

/**
 * Gets whether or not the left analog stick is depressed.
 *
 * @return The state of the left analog stick button.
 */
bool Gamepad::GetLeftPush()
{
    return GetNumberedButton(kLeftAnalogStickButton);
}

/**
 * Gets whether or not the right analog stick is depressed.
 *
 * @return The state of the right analog stick button.
 */
bool Gamepad::GetRightPush()
{
    return GetNumberedButton(kRightAnalogStickButton);
}

Gamepad::DPadDirection Gamepad::GetDPad()
{
    float x = GetRawAxis(kDPadXAxisNum);
    float y = GetRawAxis(kDPadYAxisNum);

    if (x < -0.5 && y < -0.5)
        return kUpLeft;
    if (x < -0.5 && y > 0.5)
        return kDownLeft;
    if (x > 0.5 && y > 0.5)
        return kDownRight;
    if (x > 0.5 && y < -0.5)
        return kUpRight;
    if (y < -0.5)
        return kUp;
    if (x < -0.5)
        return kLeft;
    if (y > 0.5)
        return kDown;
    if (x > 0.5)
        return kRight;

  return kCenter;
}

