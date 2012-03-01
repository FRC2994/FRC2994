#ifndef CAMERALIGHT_H_
#define CAMERALIGHT_H_

#include "Solenoid.h"
#include "DigitalOutput.h"

/**
 * CameraLight class.
 * This class is used to control the two concentric LED light rings
 * surrounding the vision system camera's lens.
 *
 * There are different hardware configurations possible, so a base
 * class is used to define a standard interface, and two derived
 * classes are used to implement the hardware specific code.
 * Calling code would normally allocate one of the derived classes,
 * but manipulate the object via base class pointers.
 * 
 * The base class is called CameraLight.
 * 
 * The first derived class is SolenoidCameraLight.  This class assumes
 * the inner and outer LED rings are each controlled by a separate
 * solenoid output.
 * 
 * The second derived class is GPIOCameraLight.  This class assumes
 * the inner and outer LED rings are both controlled by a single
 * GPIO.
 * 
 */

class CameraLight
{
public:
	typedef enum {
		kDark = 0,
		kFullBrightness = 100
	} LEDBrightness;
	
	virtual void Set(UINT32 bright = kFullBrightness) = 0;
	virtual void On() = 0;
	virtual void Off() = 0;

};

class SolenoidCameraLight: public CameraLight
{
public:
	// Brightness levels supported by Solenoid controlled
	// camera LEDs.
	typedef enum {
		kThreshold1 = 25,
		kThreshold2 = 50
	};
	
	SolenoidCameraLight(UINT32 innerLED, UINT32 outerLED);
	SolenoidCameraLight(UINT8 innerLEDModuleNumber, UINT32 innerLED,
						UINT8 outerLEDModuleNumber, UINT32 outerLED);
	SolenoidCameraLight(Solenoid *innerLED, Solenoid *outerLED);
	SolenoidCameraLight(Solenoid &innerLED, Solenoid &outerLED);
	
	virtual ~SolenoidCameraLight();

	void Set(UINT32 bright = kFullBrightness);
	void On();
	void Off();
	
private:
	Solenoid *m_innerLED;
	Solenoid *m_outerLED;
	bool m_allocatedLED;
};

class GPIOCameraLight: public CameraLight
{
public:
	GPIOCameraLight(UINT32 LED);
	GPIOCameraLight(UINT8 LEDModuleNumber, UINT32 LED);
	GPIOCameraLight(DigitalOutput *LED);
	GPIOCameraLight(DigitalOutput &LED);
	
	virtual ~GPIOCameraLight();

	void Set(UINT32 bright = kFullBrightness);
	void On();
	void Off();
	
private:
	DigitalOutput *m_LED;
	bool m_allocatedLED;
};


#endif
