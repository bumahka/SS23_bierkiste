#ifndef PINS_H
#define PINS_H

#include "Arduino.h"

// Pin Belegung
#define out_driveDirection  0
#define out_driveThrottle   4
#define out_brake           27

#define sensorLeft_trigger     23
#define sensorLeft_echo        22
#define sensorRight_trigger 19
#define sensorRight_echo    18
#define sensorMiddle_trigger     5
#define sensorMiddle_echo        21
#define sensorSpeed			35
#define rx_pin              16
#define tx_pin              17
#define N_STEERING_DIRECTION 32
#define P_STEERING_DIRECTION 33
#define N_STEERING_PULSE    25
#define P_STEERING_PULSE    26

#define INTERRUPT_BREAK 	2

#endif