#include "CameraLight.h"

/**
 * Create an instance of a Solenoid controlled LED camera light using the default
 * module
 * @param innerLED The channel number of the solenoid that controls the inner ring.
 * @param outerLED The channel number of the solenoid that controls the outer ring.
 */
SolenoidCameraLight::SolenoidCameraLight(UINT32 innerLED, UINT32 outerLED)
{
	m_innerLED = new Solenoid(innerLED);
	m_outerLED = new Solenoid(outerLED);
	m_allocatedLED = true;
	
	Off();
}

/**
 * Create an instance of a Solenoid controlled LED camera light using the given
 * module
 * @param innerLEDModuleNumber The solenoid module where the inner ring's
 * solenoid is located.
 * @param innerLED The channel number of the solenoid that controls the inner ring.
 * @param innerLEDModuleNumber The solenoid module where the outer ring's
 * solenoid is located.
 * @param outerLED The channel number of the solenoid that controls the outer ring.
 */
SolenoidCameraLight::SolenoidCameraLight(UINT8 innerLEDModuleNumber, UINT32 innerLED,
					UINT8 outerLEDModuleNumber, UINT32 outerLED)
{
	m_innerLED = new Solenoid(innerLEDModuleNumber, innerLED);
	m_outerLED = new Solenoid(innerLEDModuleNumber, outerLED);
	m_allocatedLED = true;
	
	Off();
}

/**
 * Create an instance of a Solenoid controlled LED camera light using previously
 * allocated Solenoid objects.
 * @param innerLED A pointer to the Solenoid object that controls the inner ring.
 * @param outerLED A pointer to the Solenoid object that controls the outer ring.
 */
SolenoidCameraLight::SolenoidCameraLight(Solenoid *innerLED, Solenoid *outerLED)
{
	m_innerLED = innerLED;
	m_outerLED = outerLED;
	m_allocatedLED = false;
	
	Off();
}

/**
 * Create an instance of a Solenoid controlled LED camera light using previously
 * allocated Solenoid objects.
 * @param innerLED A reference to the Solenoid object that controls the inner ring.
 * @param outerLED A reference to the Solenoid object that controls the outer ring.
 */
SolenoidCameraLight::SolenoidCameraLight(Solenoid &innerLED, Solenoid &outerLED)
{
	m_innerLED = &innerLED;
	m_outerLED = &outerLED;
	m_allocatedLED = false;
	
	Off();
}

/**
 * Destructor for the Solenoid controlled LED camera light.
 * Turn off the LEDs, and if Solenoid objects were allocated for the inner and outer
 * LEDs, delete them now.
 */
SolenoidCameraLight::~SolenoidCameraLight()
{
	Off();

	if (m_allocatedLED)
	{
		delete m_innerLED;
		delete m_outerLED;
	}
}

/**
 * Set the camera light to the given brightness.
 * @param bright The brightness of the camera light, as a percentage of maximum brightness.
 * For the Solenoid camera light, either LED ring can be fully on or off, in any combination.
 * If bright is set to 0, both LED rings are turned off.
 * If bright is set to 100, both LED rings are turned on.
 * Intermediate values of bright are implemented by turing only one of the LED rings on.
 * Note that the outer ring is a bit brighter than the inner ring.
 */
void SolenoidCameraLight::Set(UINT32 bright)
{
	if (bright == kDark) {
		m_innerLED->Set(false);
		m_outerLED->Set(false);
		return;
	}
	
	if (bright >= kFullBrightness) {
		m_innerLED->Set(true);
		m_outerLED->Set(true);
		return;
	}
	
	if ((bright > kDark) && (bright <= kThreshold1)) {
		m_innerLED->Set(true);
		m_outerLED->Set(false);
		return;
	}
	
	if ((bright > kThreshold1) && (bright <= kThreshold2)) {
		m_innerLED->Set(false);
		m_outerLED->Set(true);
		return;
	}
}
/**
 * Set the camera light fully on or fully off.
 * These methods are provided for convenience when simple on/off behaviour is required.
 * These methods just call Set() with the appropriate brightness value.
 */
void SolenoidCameraLight::On()
{
	Set(kFullBrightness);
}

void SolenoidCameraLight::Off()
{
	Set(kDark);
}


/**
 * Create an instance of a GPIO controlled LED camera light using the default
 * module
 * @param innerLED The channel number of the GPIO that controls the inner ring.
 */
GPIOCameraLight::GPIOCameraLight(UINT32 LED)
{
	m_LED = new DigitalOutput(LED);
	m_allocatedLED = true;
	
	Off();
}

/**
 * Create an instance of a GPIO controlled LED camera light using the given
 * module
 * @param LEDModuleNumber The digital module where the inner ring's
 * solenoid is located.
 * @param LED The channel number of the GPIO that controls the inner ring.
 */
GPIOCameraLight::GPIOCameraLight(UINT8 LEDModuleNumber, UINT32 LED)
{
	m_LED = new DigitalOutput(LEDModuleNumber, LED);
	m_allocatedLED = true;
	
	Off();
}

/**
 * Create an instance of a GPIO controlled LED camera light using a previously
 * allocated DigitalOutput object.
 * @param innerLED A pointer to the DigitalOutput object that controls the inner ring.
 * @param outerLED A pointer to the Solenoid object that controls the outer ring.
 */
GPIOCameraLight::GPIOCameraLight(DigitalOutput *LED)
{
	m_LED = LED;
	m_allocatedLED = false;
	
	Off();
}

/**
 * Create an instance of a GPIO controlled LED camera light using a previously
 * allocated GPIO object.
 * @param innerLED A reference to the DigitalOutput object that controls the inner ring.
 */
GPIOCameraLight::GPIOCameraLight(DigitalOutput &LED)
{
	m_LED = &LED;
	m_allocatedLED = false;
	
	Off();
}

/**
 * Destructor for the GPIO controlled LED camera light.
 * Turn off the LEDs, and if a DigitalOutput object was allocated for the
 * LED, delete it now.
 */
GPIOCameraLight::~GPIOCameraLight()
{
	Off();

	if (m_allocatedLED)
	{
		delete m_LED;
	}
}

/**
 * Set the camera light to the given brightness.
 * @param bright The brightness of the camera light, as a percentage of maximum brightness.
 * For the GPIO camera light, both LED rings are either fully on or fully off.  So a brightness
 * of 0 turns off both LED rings, and any other value turns on both LED rings.
 */
void GPIOCameraLight::Set(UINT32 bright)
{
	if (bright == kDark) {
		m_LED->Set(false);
		return;
	} else {
		m_LED->Set(true);
		return;
	}
}

/**
 * Set the camera light fully on or fully off.
 * These methods are provided for convenience when simple on/off behaviour is required.
 * These methods just call Set() with the appropriate brightness value.
 */
void GPIOCameraLight::On()
{
	Set(kFullBrightness);
}

void GPIOCameraLight::Off()
{
	Set(kDark);
}
