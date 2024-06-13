#include "spdsensor.h"

Spdsensor::Spdsensor(uint8_t pSpeedCounterPin, QuickPID &pPID):	SpeedCounterPin(pSpeedCounterPin),
																				myPID(pPID)
{

	CounterUnit.initialise(SpeedCounterPin, PCNT_PIN_NOT_USED);								//Set up counter and assign pin, set control pin as unused
	CounterUnit.set_mode(PCNT_COUNT_INC, PCNT_COUNT_DIS, PCNT_MODE_KEEP, PCNT_MODE_KEEP);	//Set count behaviour, count up on rising edge, do nothing on falling edge, low level and high level
	CounterUnit.set_event_value(PCNT_EVT_H_LIM, PCNT_HIGH_LIMIT);							//Set high limit for counter, resets counter to 0 and can trigger interrupt if tested
	CounterUnit.set_event_value(PCNT_EVT_L_LIM, PCNT_LOW_LIMIT);							//Set low limit for counter, resets counter to 0 and can trigger interrupt if tested

}

void Spdsensor::startCounter() {

	CounterUnit.clear();
	CounterUnit.resume();

	spdcounter = 0;
	spdindex = 0;
	startdetector = 0;

}

void Spdsensor::getSpeed(float &lSetpoint, float &lInput, float &lOutput, uint16_t &pSpeed) {

	/*Counter read out should happen every 160ms, or every 8th loop iteration to properly distinguish low speeds
	spdindex keeps track of loop iterations
	startdetector handles the first loop after startup were no output from the controller is available*/

	if (spdindex < 7) {
	
		//Downtime loops in which the speed isn't checked and the PID controller doesn't run
		if (startdetector == 0) {
			//On the first loop after startup clear the counter in case the Bierkiste was moved between power on and start
			CounterUnit.clear();
			CounterUnit.resume();
			startdetector++;
		}
		else if (startdetector < 7) {
			startdetector++;
		}
		else {
			/*After the first 8 loops set speed to the latest controller output during downtime loops,
			otherwise output will be overwritten by the set speed sent from the Surface every loop that doesn't run the controller*/
			pSpeed = ControllerOutput;
		}
		spdindex++;
	
	}
	else {

		//Every 8th loop read current count from the Counter
		spdcounter = CounterUnit.get_value();

		/* Convert counted events into a PWM Integer speed value for calculation 
		Equation is modeled after the equation the surface uses to convert slider values to speed values, see surface code 
		Currently set so 10% slider value on the surface should result in 1km/h speed
		Adjust final speed with the number under the slash */
		calcSpeed = (float)((1.7/1)*(spdcounter) + 91.6);

		// Set target and input speed for the controller
		lSetpoint = (float)pSpeed;
   		lInput = calcSpeed;
		
		/* Run the PID controller for a new output
		The surface can send special speed values for total stop 
		If the ESP receives these values set the speed and skip the controller
		Otherwise run the controller */
		if (lSetpoint == 0) {
			pSpeed = 0;
		} else if (lSetpoint == 70) {
			pSpeed = 70;
		} else {
			myPID.Compute();
			pSpeed = (uint16_t)lOutput;
		}

		// Save current output speed for downtime loops
		ControllerOutput = pSpeed;
		

		//Clear counter and reset loop index
		CounterUnit.clear();
		CounterUnit.resume();
		spdindex = 0;
	}
}