// https://www.pololu.com/product/2490
// modified from https://github.com/pololu/vl53l0x-arduino/blob/master/examples/Continuous/Continuous.ino

/// wiring:
// Arduino   VL53L0X board
// -------   -------------
//      5V - VIN
//     GND - GND
//     SDA - SDA
//     SCL - SCL

/// usage:
// #include "height.h"
// heightsensor::init(address);
// float offset = heightsensor::calibrate_offset();
// float height = heightsensor::read();

#include <Wire.h>
#include "VL53L0X.h"

namespace heightsensor
{
	VL53L0X sensor;

	void init()
	{
		// Wire = pins 18, 19 = sda0, scl0
		Wire.begin();
		sensor.setBus(&Wire);

		while (!sensor.init())
		{
			Serial.println("heightsensor: failed to initialize - retrying...");
			delay(1000);
		}

		sensor.setTimeout(500);
		// Start continuous back-to-back mode (take readings as
		// fast as possible).  To use continuous timed mode
		// instead, provide a desired inter-measurement period in
		// ms (e.g. sensor.startContinuous(100)).
		sensor.startContinuous(/*200000*/);

		// reduce timing budget to 20 ms (default is about 33 ms)
		bool valid = sensor.setMeasurementTimingBudget(20000);
		if (!valid)
		{
			Serial.println("heightsensor: invalid measurement timing budget");
		}
	}

	float calibrate_offset()
	{
		float lidaroffset = 0;
		int nloops = 16;
		for (int k = 0; k < nloops; k++)
		{
			lidaroffset += sensor.readRangeContinuousMillimeters() / nloops;
		}
		return lidaroffset;
	}

	float read()
	{
		float reading = sensor.readRangeContinuousMillimeters();

		if (sensor.timeoutOccurred())
		{
			Serial.print("heightsensor: timeout occurred");
		}
		return reading;
	}
} // namespace heightsensor