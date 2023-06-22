#include <Arduino.h>
#include <AccelStepper.h> 
#include <iostream>
#include <string> 
#include <Ultrasonic.h>
#include <sensors.cpp>

#include "uart.cpp"
#include "antriebsmotor.h"
#include "pins.h"


AccelStepper stepper(AccelStepper::FULL4WIRE,33,32,25,14,true);  // 4 wire full stepper
long steering_val=0;


Ultrasonic sensor1(sensor1_trigger, sensor1_echo);
Ultrasonic sensor2(sensor2_trigger, sensor2_echo);
Ultrasonic sensor3(sensor3_trigger, sensor3_echo);

void setup() {
  SerialPort.begin(15200, SERIAL_8N1, 16, 17); //using pin 16 and 17 on ESP (Baudrate, SerialMode, RX_pin, TX_pin)

  /*
  Hier evtl. IN- bzw. OUTPUTS deklarieren, sollte jedoch in der AccelStepper Library automatisch passieren
  */
  stepper.setAcceleration(5000);        // Acceleration in steps/s^2
  stepper.setMaxSpeed(5000);            // Speed in steps/s
  stepper.setSpeed(2000);
  stepper.enableOutputs();
  //stepper.setOutputPins(AccelStepper::FULL4WIRE);
  stepper.setMinPulseWidth(20);         // 20 microseconds

}

void loop() 
{
  buildMessage(directionValue, speedValue, steeringValue, distance1Value, distance2Value, distance3Value);
  Serial.print(msgUart);
  //getMessage((byte)msg); //string to byte conversion?? in what format is uart sent and received??
  // put your main code here, to run repeatedly:
  // sensors[0] = sensor1.read(); //Reads distance in cm
  // sensors[1] = sensor2.read();
  // sensors[2] = sensor3.read();

  distanceCheckEasy(emergencyBreakValue, maximalDistanceValue, sensor1.read());
  distanceCheckEasy(emergencyBreakValue, maximalDistanceValue, sensor2.read());
  distanceCheckEasy(emergencyBreakValue, maximalDistanceValue, sensor3.read());

  delay(1000); //pause für 1 s, determines the rate of distance calculation. Can be changed eventually (mimum 10)

  SerialPort.write(msgUart); //uses uart_write_bytes()
  
  //Receive message
  if(Serial.available()>0){
    uint8_t c = Serial.read(); //uses uart_read_bytes()
    //should be the byte that was send
    //how do we extract the bits out of the integer???
    getMessage(c);  
  }
  //if(uart.any())
  //{ 
      stepper.moveTo(steering_val*(-10)); // negative anticlockwise
      while(stepper.distanceToGo()!=0)
      {
        stepper.run();
      }
  //}
} 

  // for(int i = 0; i<3; i++){
  //   if(distanceCheck(emergencyBreakValue, i) == false){
  //     //before interrupting check value again to avoid ghosts
  //     //interrupt, emergency break!
  //     //to implement: motor stop (up to date emergency stop)
  //     printf("EMERGENCY BREAK");
  //   } //else continue;
  // }