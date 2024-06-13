#ifndef ANTRIEBSMOTOR_H
#define ANTRIEBSMOTOR_H

#include <Arduino.h>

#define PWM_FREQUENCY 5000
#define PWM_CHANNEL 2
#define PWM_RESOLUTION 8
#define SAVE_STATE 40
const uint16_t MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION)-1);


class antrieb {
private:
    const uint8_t mThrottlePin;
    const uint8_t mDirectionPin;

    uint16_t mSpeed;
    byte mDirection;
        /*  TRUE = backwards 
         *  FALSE = forward  */
    
public:
    antrieb(uint8_t pThrottlePin, uint8_t pDirectionPin);
    boolean setDirection(byte &pDirection); 
    boolean setSpeed(uint16_t &pSpeed);
    boolean setSaveState();
    uint16_t getSpeed();
};

#endif