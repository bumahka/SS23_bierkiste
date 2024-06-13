#include "sensors.h"

sensor::sensor(uint8_t TriggerPin1, uint8_t TriggerPin2,
               uint8_t TriggerPin3, uint8_t EchoPin1,
               uint8_t EchoPin2, uint8_t EchoPin3, Break &pBreak) : sensorTriggerPin1(TriggerPin1), sensorTriggerPin2(TriggerPin2),
                                                                    sensorTriggerPin3(TriggerPin3), sensorEchoPin1(EchoPin1),
                                                                    sensorEchoPin2(EchoPin2), sensorEchoPin3(EchoPin3), myBreak(pBreak),
                                                                    sensorLeft(sensorTriggerPin1, sensorEchoPin1),
                                                                    sensorRight(sensorTriggerPin2, sensorEchoPin2),
                                                                    sensorMiddle(sensorTriggerPin3, sensorEchoPin3),
                                                                    mIndex(0)
{
  pinMode(sensorLeft_trigger, OUTPUT);
  pinMode(sensorLeft_echo, INPUT);
  pinMode(sensorRight_trigger, OUTPUT);
  pinMode(sensorRight_echo, INPUT);
  pinMode(sensorMiddle_trigger, OUTPUT);
  pinMode(sensorMiddle_echo, INPUT);

  for (uint8_t i = 0; i < FILTER_SIZE; i++)
  {
    distanceLeftFilter[i] = sensorLeft.read();
    distanceRightFilter[i] = sensorRight.read();
    distanceMiddleFilter[i] = sensorMiddle.read();
  }
}

void sensor::setDistance(uint16_t pDistance, uint8_t sensorIndex)
{
  if (pDistance == 0)
  {
    Serial.print("Sensor ");
    Serial.print(sensorIndex);
    Serial.println(" not working!");
  }
  switch (sensorIndex)
  {
  case 1:
    distanceLeftFilter[mIndex] = pDistance;
    distanceLeft = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
      distanceLeft += distanceLeftFilter[i];
    }
    distanceLeft = distanceLeft / FILTER_SIZE;

    Serial.print("Sensor 1: ");
    Serial.println(distanceLeft);

    break;
  case 2:
    distanceRightFilter[mIndex] = pDistance;
    distanceRight = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
      distanceRight += distanceRightFilter[i];
    }
    distanceRight = distanceRight / FILTER_SIZE;

    Serial.print("Sensor 2: ");
    Serial.println(distanceRight);

    break;
  case 3:
    distanceMiddleFilter[mIndex] = pDistance;
    distanceMiddle = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
      distanceMiddle += distanceMiddleFilter[i];
    }
    distanceMiddle = distanceMiddle / FILTER_SIZE;

    Serial.print("Sensor 3: ");
    Serial.println(distanceMiddle);

    break;
  default:
    // do nothing for now
    break;
  }
}

/* read the distance and break if necessary*/
void sensor::readDistance()
{
  setDistance(sensorLeft.read(), 1);
  setDistance(sensorRight.read(), 2);
  setDistance(sensorMiddle.read(), 3);
  mIndex++;
  mIndex %= FILTER_SIZE;

  if (!distanceOK())
    myBreak.Activate_EmergencyBreak();
}

uint16_t sensor::getDistance(uint8_t sensorIndex)
{
  switch (sensorIndex)
  {
  case 1:
    return distanceLeft;
  case 2:
    return distanceMiddle;
  case 3:
    return distanceRight;
  default:
    return 0; /* might cause an emergency break */
  }
}

// check if distance is below emergency break distance.
bool sensor::distanceOK()
{
  bool distanceState = true;
  for (int i = 1; i < 4; i++)
  {
    if (getDistance(i) < EMERGENCY_DISTANCE_VALUE)
    {
      distanceState &= false;
    }
  }
  return distanceState;
}

// to check if sensors are correct (triplet-system based)
// to be implemented in further steps
bool sensor::securityCheck(uint8_t sensorIndex)
{
  return true;
}
