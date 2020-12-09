#include <Arduino.h>
#include <Wire.h>
#include "boom.h"
#include "imu.h"
#include "height.h"

// 2000 micro seconds -> 500 Hz loop time
// if you enconter problems, slow down!
unsigned long looptime_us = 2000;
unsigned long prevtime_us = 0;
unsigned long evaltime_us = 0;

// comms to laptop
#define Laptop Serial
const uint32_t laptop_baud = 250000;
void sendfloat(float);
void sendfloats(float *, size_t);
void wait_control_loop();

void setup()
{
	Laptop.flush();
	Laptop.begin(laptop_baud);
	Laptop.write("teensy-laptop comms working");

	heightsensor::init(0x69); // address?
	imu::init();			  // address?
	boom::init();			  // pins?

	// TODO: initial state? calculate offsets? a button?
}

void loop()
{
	// header: 2 bytes
	Laptop.write((uint8_t)0xAA);
	Laptop.write((uint8_t)0x55);

	// dummy data
	// sendfloat(1.5);
	// sendfloat(3.6);
	// float _data[9] = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9};
	// sendfloats(_data, 9);
	// delay(1000);

	// data: 4 + 4 + 9*4 bytes
	sendfloat(boom::read());
	sendfloat(heightsensor::read());
	imu::read();
	sendfloats(imu::data, 9);
	wait_control_loop();
}

void wait_control_loop()
{
	evaltime_us = prevtime_us + looptime_us;
	while (micros() < evaltime_us)
		;
	prevtime_us = micros();
}

void sendfloat(float data)
{
	Laptop.write((uint8_t *)&data, sizeof(data));
}

void sendfloats(float *data, uint n)
{
	for (uint i = 0; i < n; i++)
	{
		sendfloat(data[i]);
	}
}
