#ifndef BREMSE_H
#define BREMSE_H

#include <Arduino.h>
#include "pins.h"
#include "antriebsmotor.h"

#define BREAK_PWM_FREQUENCY 1000
#define BREAK_PWM_CHANNEL 0
#define BREAK_PWM_RESOLUTION 8
#define BREAK_SET 120
#define BREAK_LOOSE 255
class Break{

private:
    
    boolean EmergencyBreakActive;
    boolean pEmergencyBreakActive;

    antrieb &myAntrieb;
    SemaphoreHandle_t &mySemaphore;

public:
    
    Break(antrieb &pAntrieb, SemaphoreHandle_t &pSemaphore);

    void Activate_EmergencyBreak();
    void Deactivate_EmergencyBreak();
    boolean get_State_Break();

};

#endif