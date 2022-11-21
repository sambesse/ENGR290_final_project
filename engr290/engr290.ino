#include <Arduino.h>
#include "TWI.h"
#include "sensors.h"
#include "timing.h"

typedef struct {
  uint16_t leftSensorData;
  uint16_t rightSensorData;
  uint16_t frontSensorData;
  uint16_t yawRate;
} SensorData;

enum State {STRAIGHT, TURNING}; 

State state = STRAIGHT;
USSensorData frontSensor;
ADCData leftSensor;
ADCData rightSensor;
SensorData rawData;
uint8_t semaphore = 0;

void setup() {
  initIMU();
  initIRSensors();
  initFrontSensor(&frontSensor);
}

void loop() {
  if(semaphore & IR_SEMAPHORE) {
    startIRReading(&leftSensor);
    semaphore &= ~IR_SEMAPHORE; 
  }
  if(semaphore & IMU_SEMAPHORE) {
    requestTWI(GYRO_YAW_START, 2);
    semaphore &= ~IMU_SEMAPHORE;
  }
  if(leftSensor.semaphore & DATA_READY) {
    startIRReading(&rightSensor);
    rawData.leftSensorData = leftSensor.adcResult;
    leftSensor.semaphore &= ~DATA_READY;
  }
  if(semaphore & TWI_DATA_READY_SEMAPHORE) {
    receiveTWI(&rawData.yawRate);
    semaphore &= TWI_DATA_READY_SEMAPHORE;
  }
  if(rightSensor.semaphore & DATA_READY) {
    rawData.rightSensorData = rightSensor.adcResult;
    rightSensor.semaphore &= ~DATA_READY;
  }
  if(frontSensor.semaphore & DATA_READY) {
    rawData.frontSensorData = frontSensor.pulseLength;
    frontSensor.semaphore &= ~DATA_READY;
  }
}
