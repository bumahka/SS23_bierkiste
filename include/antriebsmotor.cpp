#include "antriebsmotor.h"

antrieb::antrieb(uint8_t pThrottlePin, uint8_t pDirectionPin):
    mThrottlePin(pThrottlePin), mDirectionPin(pDirectionPin), mSpeed(270), mDirection(0){
    /* PWM setup */
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    /* attach pwm channel to GPIO pin */
    ledcAttachPin(mThrottlePin, PWM_CHANNEL);
    /* Direction Pin setup */
    pinMode(mDirection, OUTPUT);
    /* set motor to speed 0 */
    setSpeed(mSpeed);
    /* set direction to forward */
    setDirection(mDirection);    
}

boolean antrieb::setDirection(byte &pDirection){
    mDirection = pDirection;
    digitalWrite(mDirectionPin, mDirection);
    return true;
}

boolean antrieb::setSpeed(uint16_t &pSpeed){
    mSpeed = pSpeed;
    //Serial.println(mSpeed);
    ledcWrite(PWM_CHANNEL, mSpeed);
    return true;
}

boolean antrieb::setSaveState(){
    mDirection= 0;
    setDirection(mDirection);

    mSpeed = SAVE_STATE;
    setSpeed(mSpeed);
    return true;
}

