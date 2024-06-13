#ifndef SPDSENSOR_H
#define SPDSENSOR_H

#include <Arduino.h>
#include "pins.h"
#include <esp32_pcnt.h>
#include <QuickPID.h>

/*Class to define the speed sensor to measure current speed
	contains:
	Initialisation with HW counter setup
	Counter start
	Speed read-out including conversion from counter into 16bit value and PID control*/

#define PCNT_HIGH_LIMIT 200
#define PCNT_LOW_LIMIT -1

class Spdsensor {

private:

	const uint8_t SpeedCounterPin;

	uint16_t spdcounter;
	uint8_t spdindex;
	uint8_t startdetector;
	float calcSpeed;
	uint16_t ControllerOutput;
	
	PulseCounter CounterUnit;

	QuickPID &myPID;

public:

	Spdsensor(uint8_t pSpeedCounterPin, QuickPID &pPID);
	void startCounter();
	void getSpeed(float &lSetpoint, float &lInput, float &lOutput, uint16_t &pSpeed);
};
#endif