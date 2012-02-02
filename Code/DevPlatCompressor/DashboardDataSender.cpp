#include "DashboardDataSender.h"
#include "WPILib.h"

/**
 * Send data to the dashboard.
 * This class sends two types of data to the dashboard program:
 * 1. Data representing all the ports on the robot
 * 2. Camera tracking data so the dashboard can annotate the video stream with
 *    target information.
 */
DashboardDataSender::DashboardDataSender()
{
	// these timers make sure that the data is not sent to the dashboard more
	// than 10 times per second for efficiency.
	IOTimer = new Timer();
	IOTimer->Start();
}

/**
 * Send the vision tracking data.
 * Sends the vision information to the dashboard so that the images will be annotated
 * and the graphs will operate.
 */

/**
 * Send IO port data to the dashboard.
 * Send data representing the output of all the IO ports on the cRIO to the dashboard.
 * This is probably not the best data to send for your robot. Better would be higher
 * level information like arm angle or collector status. But this is a sample and you're
 * free to modify it. Be sure to make the corresponding changes in the LabVIEW example
 * dashboard program running on your driver station.
 */
void DashboardDataSender::sendIOPortData(Solenoid * sol3) {
	if (IOTimer->Get() < 0.1)
		return;
	IOTimer->Reset();
	Dashboard &dash = DriverStation::GetInstance()->GetLowPriorityDashboardPacker();
	unsigned char solBuf = 0;
	dash.AddCluster();
	{
		dash.AddCluster();
		{ //analog modules 
			dash.AddCluster();
			{
				for (int i = 1; i <= 8; i++) {
					dash.AddFloat((float) AnalogModule::GetInstance(1)->GetAverageVoltage(i));
//					dash.AddFloat((float) i * 5.0 / 8.0);
				}
			}
			dash.FinalizeCluster();
//			dash.AddCluster();
//			{
//				for (int i = 1; i <= 8; i++) {
//					dash.AddFloat((float) AnalogModule::GetInstance(2)->GetAverageVoltage(i));
//				}
//			}
//			dash.FinalizeCluster();
		}
		dash.FinalizeCluster();

		dash.AddCluster();
		{ //digital modules
			dash.AddCluster();
			{
				dash.AddCluster();
				{
					int module = 2;
					UINT16 value = 0;
					dash.AddU8(DigitalModule::GetInstance(module)->GetRelayForward());
					dash.AddU8(DigitalModule::GetInstance(module)->GetRelayReverse());
					
					//dash.AddU16((short)DigitalModule::GetInstance(module)->GetDIO());
					//dash.AddU16((short) 0xAAAA);
					for (int c = 1 ; c < 15 ; c++)
					{
						value |= (DigitalModule::GetInstance(module)->GetDIO(c) << (c - 1));
					}
					dash.AddU16((short)value);
					
					//dash.AddU16((short)DigitalModule::GetInstance(module)->GetDIODirection());
					//dash.AddU16((short) 0x7777);
					for (int c = 1 ; c < 15 ; c++)
					{
						value |= (DigitalModule::GetInstance(module)->GetDIODirection(c) << (c - 1));
					}
					dash.AddU16((short)value);
					dash.AddCluster();
					{
						for (int i = 1; i <= 10; i++) {
							dash.AddU8((unsigned char) DigitalModule::GetInstance(module)->GetPWM(i));
							//dash.AddU8((unsigned char) (i-1) * 255 / 9);
						}
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster();
			}
			dash.FinalizeCluster();

			dash.AddCluster();
			{
				dash.AddCluster();
				{
					int module = 6;
					dash.AddU8(DigitalModule::GetInstance(module)->GetRelayForward());
					dash.AddU8(DigitalModule::GetInstance(module)->GetRelayForward());
					dash.AddU16((short)DigitalModule::GetInstance(module)->GetDIO());
					dash.AddU16(DigitalModule::GetInstance(module)->GetDIODirection());
					dash.AddCluster();
					{
						for (int i = 1; i <= 10; i++) {
							dash.AddU8((unsigned char) DigitalModule::GetInstance(module)->GetPWM(i));
							//dash.AddU8((unsigned char) i * 255 / 10);
						}
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster();
			}
			dash.FinalizeCluster();
		}
		dash.FinalizeCluster();

		// Can't read solenoids without an instance of the object
		// dash.AddU8((char) 0);
		
		// Get the solenoids as an 8-bit int
		dash.AddCluster();
		{
//			for(unsigned i=1;i<8;i++) {
//				solBuf |= (Solenoid(i).Get() << (i-1));
//			}
			solBuf |= sol3->Get() << 7;
//			dash.AddU8(0x55);
			dash.AddU8(solBuf);
		}
		dash.FinalizeCluster();

	}
	dash.FinalizeCluster();
	dash.Finalize();
}
