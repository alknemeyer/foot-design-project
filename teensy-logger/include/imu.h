// usage:
// #include "imu.h"
// imu::init();
// imu::read();
// // -> `imu::data` is an array of 9 floats: acc xyz, gyro xyz, mag xyz

#include <Wire.h>
#include "MPU9250.h"

namespace imu
{
	// Wire1 = pins 16, 17 = scl1, sda1
	MPU9250 imu(Wire1, 0x68);
	int status;
	float data[9] = {0};

	void init()
	{
		Wire1.begin();

		while ((status = imu.begin()) < 0)
		{
			Serial.print("mpu: failed to initialize. Got status = ");
			Serial.print(status);
			Serial.print(". Check IMU wiring or try cycling power - retrying...");
			delay(1000);
		}

		// setting the accelerometer full scale range to +/-8G
		imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
		// setting the gyroscope full scale range to +/-500 deg/s
		imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
		// setting DLPF bandwidth to 20 Hz
		imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
		// setting SRD to 19 for a 50 Hz update rate
		imu.setSrd(19);
	}

	void read()
	{
		imu.readSensor();
		data[0] = imu.getAccelX_mss();
		data[1] = imu.getAccelY_mss();
		data[2] = imu.getAccelZ_mss();
		data[3] = imu.getGyroX_rads();
		data[4] = imu.getGyroY_rads();
		data[5] = imu.getGyroZ_rads();
		data[6] = imu.getMagX_uT();
		data[7] = imu.getMagY_uT();
		data[8] = imu.getMagZ_uT();
	}

} // namespace imu
