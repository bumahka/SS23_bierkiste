#include "uart_message.h"

UartMessage::UartMessage(uint8_t rx_pin, uint8_t tx_pin, uint64_t sBaudrate, uint64_t sSerialMode) : uart_rx_pin(rx_pin), uart_tx_pin(tx_pin), baudrate(sBaudrate), serialMode(sSerialMode)
{

    Serial2.begin(baudrate, serialMode, uart_rx_pin, uart_tx_pin);

    direction = 0;
    speed = 270;
    distance_1 = 100;
    distance_2 = 120;
    distance_3 = 140;
}

void UartMessage::getInstructions()
{
    /* Polling for first byte of the message */
    if (Serial2.read() == MESSAGE_ID)
    {
        /* reading the message */
        Serial2.readBytes(rxBuffer, 5);

        /* Asigning the received data */
        /* Drive direction */
        direction = rxBuffer[0];

        /* Speed */
        byte *speed_ptr = (byte *)&speed;
        speed_ptr[0] = rxBuffer[1];
        speed_ptr[1] = rxBuffer[2];

        /* Steering */
        byte *steering_ptr = (byte *)&steering;
        steering_ptr[0] = rxBuffer[3];
        steering_ptr[1] = rxBuffer[4];
    }
}

void UartMessage::getDirection(byte &pDirection)
{
    pDirection = direction;
}

void UartMessage::getSpeed(uint16_t &pSpeed)
{
    pSpeed = speed;
}

void UartMessage::getDistance(uint16_t *pDistance[3])
{
    *pDistance[0] = distance_1; 
    *pDistance[1] = distance_2;
    *pDistance[2] = distance_3;
}

void UartMessage::getSteering(int16_t &pSteering)
{
    pSteering = steering;
}
