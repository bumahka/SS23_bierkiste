#include "sensors.h"

sensor::sensor(uint8_t TriggerPin1, uint8_t TriggerPin2,uint8_t TriggerPin3,
              uint8_t EchoPin1, uint8_t EchoPin2, uint8_t EchoPin3):

    sensorTriggerPin1(TriggerPin1), sensorTriggerPin2(TriggerPin2), sensorTriggerPin3(TriggerPin3), 
    sensorEchoPin1(EchoPin1), sensorEchoPin2(EchoPin2), sensorEchoPin3(EchoPin3)
    {
}

void sensor::setDistance(uint8_t value, uint8_t number){
  switch (number) {
    case 1:
      distance1 = value;
    case 2:
      distance2 = value;
    case 3: 
      distance3 = value;
    default:
      //do nothing for now
  } 
}

void sensor::readDistance(){
  Ultrasonic sensor1(sensorTriggerPin1, sensorEchoPin1);
  Ultrasonic sensor2(sensorTriggerPin2, sensorEchoPin2);
  Ultrasonic sensor3(sensorTriggerPin3, sensorEchoPin3);
  setDistance(sensor1.read(), 1);
  setDistance(sensor2.read(), 2);
  setDistance(sensor3.read(), 3);
}

uint8_t sensor::getDistance(uint8_t number){
  switch (number) {
    case 1:
      return distance1;
    case 2:
      return distance2;
    case 3:
      return distance3;  
    default:
      return 0;
  }
}

// check if distance is below emergency break distance. 
void sensor::distanceCheck()
{
  for (int i=1; i<4; i++){
    if(getDistance(i) < emergencyDistanceValue){
      Serial.println("error ");
    }
    if(getDistance(i) > maximalDistanceValue){
      Serial.println("too far away ");
    }
  }

}

// to check if sensors are correct (triplet-system based)
// to be implemented in further steps
bool sensor::securityCheck(uint8_t sensorNumber){
}

