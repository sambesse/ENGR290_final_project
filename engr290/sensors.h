#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "TWI.h"

/*all connections
* P14 left IR sensor
* P16 right IR sensor
* P2 servo motor
* P13 front US Sensor
* P11 lift fan
* P3 thrust fan
* P17 IMU
*/

#define LEFT_IR_PIN (A0) //TODO make this correct
#define RIGHT_IR_PIN (A1) //TODO make this correct
//constants fron US sensor semaphore
#define IN_PULSE (1 << 7)
#define DATA_READY (1 << 6)

typedef struct {
  uint16_t pulseStart;
  uint16_t pulseEnd;
  uint32_t pulseLength;
  uint8_t semaphore;
} USSensorData;

typedef struct {
  uint8_t adcMux;
  uint16_t adcResult;
  uint8_t semaphore;
} ADCData;

//init function for IMU
void initIMU();

//sets up the interrupt and pointers
void initFrontSensor(USSensorData* frontSensorData); 

void calibrateIMU();

#endif
