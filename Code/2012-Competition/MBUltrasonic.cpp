/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "MBUltrasonic.h"

#include "Counter.h"
#include "DigitalInput.h"
#include "DigitalOutput.h"
#include "Timer.h"
#include "Utility.h"
#include "WPIErrors.h"
#include <stdlib.h>

const double MBUltrasonic::kPingTime;	///< Time (sec) for the ping trigger pulse.
const double MBUltrasonic::kMaxUltrasonicTime;	///< Max time (ms) between readings.
const double MBUltrasonic::kSpeedOfSoundInchesPerSec;

/**
 * Initialize the MBUltrasonic Sensor.
 * This is the common code that initializes the MBUltrasonic sensor given that there
 * are two digital I/O channels allocated.
 */
void MBUltrasonic::Initialize()
{
	m_counter = new Counter(m_echoChannel); // set up counter for this sensor
	m_counter->SetMaxPeriod(1.0);
	m_counter->SetSemiPeriodMode(true);
	m_counter->Reset();
	m_counter->Start();
	
	m_sampleTimer = new Timer();	// Timer to gather range samples at regular intervals.
	
	m_medianRange = 0;				// Initially, set median of latest samples to 0 inches.
	m_sampleCount = 0;				// Initially, no samples.
}

/**
 * Create an instance of the MBUltrasonic Sensor using the default module.
 * @param pingChannel The digital output channel that enables pings.  Pings are enabled if this
 * channel is high, disabled if it is set low.
 * @param echoChannel The digital input channel that receives the echo. The length of time that the
 * echo is high represents the round trip time of the ping, and the distance.
 * @param units The units returned in either kInches or kMilliMeters
 */
MBUltrasonic::MBUltrasonic(UINT32 pingChannel, UINT32 echoChannel, DistanceUnit units)
{
	m_pingChannel = new DigitalOutput(pingChannel);
	m_echoChannel = new DigitalInput(echoChannel);
	m_allocatedChannels = true;
	m_units = units;
	Initialize();
}

/**
 * Create an instance of an MBUltrasonic Sensor from a DigitalInput for the echo channel and a
 * DigitalOutput for the ping channel.
 * @param pingChannel The digital output channel that enables pings.  Pings are enabled if this
 * channel is high, disabled if it is set low.
 * @param echoChannel The digital input object that times the return pulse to determine the range.
 * @param units The units returned in either kInches or kMilliMeters
 */
MBUltrasonic::MBUltrasonic(DigitalOutput *pingChannel, DigitalInput *echoChannel, DistanceUnit units)
{
	if (pingChannel == NULL || echoChannel == NULL)
	{
		wpi_setWPIError(NullParameter);
		return;
	}
	m_allocatedChannels = false;
	m_pingChannel = pingChannel;
	m_echoChannel = echoChannel;
	m_units = units;
	Initialize();
}

/**
 * Create an instance of an MBUltrasonic Sensor from a DigitalInput for the echo channel and a
 * DigitalOutput for the ping channel.
 * @param pingChannel The digital output channel that enables pings.  Pings are enabled if this
 * channel is high, disabled if it is set low.
 * @param echoChannel The digital input object that times the return pulse to determine the range.
 * @param units The units returned in either kInches or kMilliMeters
 */
MBUltrasonic::MBUltrasonic(DigitalOutput &pingChannel, DigitalInput &echoChannel, DistanceUnit units)
{
	m_allocatedChannels = false;
	m_pingChannel = &pingChannel;
	m_echoChannel = &echoChannel;
	m_units = units;
	Initialize();
}

/**
 * Create an instance of the MBUltrasonic sensor using specified modules.
 * This constructor takes the channel and module slot for each of the required digital I/O channels.
 * @param pingModuleNumber The digital module that the pingChannel is on.
 * @param pingChannel The digital output channel that sends the pulse to initiate the sensor
 * sending the ping.
 * @param pingChannel The digital output channel that enables pings.  Pings are enabled if this
 * channel is high, disabled if it is set low.
 * @param echoModuleNumber The digital module that the echoChannel is on.
 * @param echoChannel The digital input channel that receives the echo. The length of time
 * that the echo is high represents the round trip time of the ping, and the distance.
 * @param units The units returned in either kInches or kMilliMeters
 */
MBUltrasonic::MBUltrasonic(UINT8 pingModuleNumber, UINT32 pingChannel,
		UINT8 echoModuleNumber, UINT32 echoChannel, DistanceUnit units)
{
	m_pingChannel = new DigitalOutput(pingModuleNumber, pingChannel);
	m_echoChannel = new DigitalInput(echoModuleNumber, echoChannel);
	m_allocatedChannels = true;
	m_units = units;
	Initialize();
}

/**
 * Destructor for the MBUltrasonic sensor.
 * Delete the instance of the MBUltrasonic sensor by freeing the allocated digital channels.
 */
MBUltrasonic::~MBUltrasonic()
{
	if (m_allocatedChannels)
	{
		delete m_pingChannel;
		delete m_echoChannel;
	}
	
	if (m_counter) {
		delete m_counter;
	}
	
	if (m_sampleTimer) {
		delete m_sampleTimer;
	}

  }


/**
 * Turn on pinging on MBUltrasonic sensor.
 * Set the EZ1's RCVR pin high, enabling ranging.
 * mode is disabled. A single ping is sent out, and the counter should count the semi-period
 * when it comes in. The counter is reset to make the current value invalid.
 */
void MBUltrasonic::PingEnable(bool enable)
{
	m_pingEnabled = enable;
	if (enable) {
		m_counter->Reset(); // reset the counter to zero (invalid data now)
		m_sampleTimer->Start();
		m_pingChannel->Set(1); // Set RCVR high, enabling repetitive pinging.
	} else {
		m_counter->Reset(); // reset the counter to zero (invalid data now)
		m_sampleTimer->Stop();
		m_pingChannel->Set(0); // Set RCVR low, disabling repetitive pinging.
	}
}

/**
 * Check if there is a valid range measurement.
 * The ranges are accumulated in a counter that will increment on each edge of the echo (return)
 * signal. If the count is not at least 2, then the range has not yet been measured, and is invalid.
 */
bool MBUltrasonic::IsRangeValid()
{
	return m_counter->Get() > 1;
}


/**
 * Get the range in inches directly from the MBUltrasonic sensor.
 * @return double Range in inches of the target returned from the MBUltrasonic sensor. If there is
 * no valid value yet, i.e. ranging isn't enabled, then return 0.
 */
double MBUltrasonic::GetRawRangeInches()
{
	if (IsRangeValid())
			return m_counter->GetPeriod() * kSpeedOfSoundInchesPerSec / 2.0;
	else
		return 0;
}

/**
 * Get the range in millimeters from the MBUltrasonic sensor.
 * @return double Range in millimeters of the target returned by the MBUltrasonic sensor.
 * If there is no valid value yet, i.e. ranging isn't enabled, then return 0.
 */
double MBUltrasonic::GetRawRangeMM()
{
	return GetRawRangeInches() * 25.4;
}

/**
 * Compare two doubles.
 * This is a helper function for UpdateRangeSamples, which
 * calls qsort.
 * @return int Difference between parameters; positive if one > two.
 * @param one The first double to compare.
 * @param two The second double to compare.
 */
int double_compare(const void *one, const void *two)
{
	return (int)(*((double*)one) - *((double*)two));
}

/**
 * Update array of range samples.
 * The raw range values are prone to transient
 * high/low values, so we need to filter them.
 * Currently, we only calculate the median of the last
 * kNumSamples.
 * This function is called repeatedly in the main loop,
 * and we use a timer to throttle how quickly we gather
 * raw samples.  Once we have enough samples, we calculate
 * the median and store the value, which can be retrieved
 * with GetRangeInches or getRangeMM.
 * @return void
 */
void MBUltrasonic::UpdateRangeSamples()
{
	// Only update samples if it has been
	// more than kRangePeriod since the last
	// update.
	if (!m_sampleTimer->HasPeriodPassed(kRangePeriod)) {
		return;
	}
	
	// Store the current raw range value in the array.
	// Only increment m_sampleCount after using it
	// as the array index.  That way, first sample
	// goes in entry 0, second in entry 1, etc.
	m_rawRange[m_sampleCount] = m_counter->GetPeriod() * kSpeedOfSoundInchesPerSec / 2.0;
	m_sampleCount++;
	
	// We need a number of samples collected
	// at regular intervals to do our filtering.
	// If we have enough samples, filter the
	// data in m_rawRange.
	if (m_sampleCount >= kNumSamples) {
		m_sampleCount = 0;
		qsort(m_rawRange, kNumSamples, sizeof(double), double_compare);
		m_medianRange = m_rawRange[kNumSamples/2];
	}
	
	// Wait kRangePeriod before adding another sample.
	m_sampleTimer->Reset();
}

/**
 * Get the range in inches from the MBUltrasonic sensor.
 * The value returned is filtered to exclude transient high/low values.
 * @return double Range in inches of the target returned by the MBUltrasonic sensor.
 * If there is no valid value yet, i.e. ranging is not enabled, then return 0.
 */

double MBUltrasonic::GetRangeInches()
{
	if (IsRangeValid())
		return m_medianRange;
	else
		return 0;
}

/**
 * Get the range in millimeters from the MBUltrasonic sensor.
 * The value returned is filtered to exclude transient high/low values.
 * @return double Range in millimeters of the target returned by the MBUltrasonic sensor.
 * If there is no valid value yet, i.e. ranging is not enabled, then return 0.
 */
double MBUltrasonic::GetRangeMM()
{
	return GetRangeInches() * 25.4;
}

/**
 * Get the range in the current DistanceUnit for the PIDSource base object.
 * 
 * @return The range in DistanceUnit
 */
double MBUltrasonic::PIDGet()
{
	switch(m_units) 
	{
	case MBUltrasonic::kInches:
		return GetRangeInches();
	case MBUltrasonic::kMilliMeters:
		return GetRangeMM();
	default:
		return 0.0;
	}
}

/**
 * Set the current DistanceUnit that should be used for the PIDSource base object.
 * 
 * @param units The DistanceUnit that should be used.
 */
void MBUltrasonic::SetDistanceUnits(DistanceUnit units)
{
	m_units = units;
}

/**
 * Get the current DistanceUnit that is used for the PIDSource base object.
 * 
 * @return The type of DistanceUnit that is being used.
 */
MBUltrasonic::DistanceUnit MBUltrasonic::GetDistanceUnits()
{
	return m_units;
}
