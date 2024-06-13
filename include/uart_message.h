#ifndef UART_MESSAGE_H 
#define UART_MESSAGE_H

#include <Arduino.h>

#define MESSAGE_ID 0xAA

class UartMessage {

private:
    const uint8_t uart_rx_pin;
    const uint8_t uart_tx_pin;
    const uint64_t baudrate;
    const uint64_t serialMode;
    byte direction;
    uint16_t speed;
    int16_t steering;
    uint16_t distance_1;
    uint16_t distance_2;
    uint16_t distance_3;
    
    //byte power;
    byte rxBuffer[5];
    byte txBuffer[7];

public:
    UartMessage(uint8_t rx_pin, uint8_t tx_pin, uint64_t sBaudrate, uint64_t sSerialMode);

    void getInstructions();
    
    void getDirection(byte &pDirection);
    void getSpeed(uint16_t &pSpeed);
    void getSteering(int16_t &pSteering);
    bool msgAvailable();
    bool availableForTransmit();
    void transmitDistances(uint16_t pDistance[3]);
    
};
#endif