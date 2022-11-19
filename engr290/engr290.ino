#include <Arduino.h>
#include "TWI.h"
#include "sensors.h"
#include "timing.h"

enum State {STRAIGHT, TURNING}; 
State state = STRAIGHT;
USSensorData frontSensor;
ADCData leftSensor;
ADCData rightSensor;

void setup() {
  initIMU();
  initIRSensors();
  initFrontSensor(&frontSensor);
  
}

void loop() {
  if(semaphore & IR_SEMAPHORE) {
    semaphore &= ~IR_SEMAPHORE; 
    startIRReading(&leftSensor);
  }
  if(semaphore & IMU_SEMAPHORE) {
    semaphore &= ~IMU_SEMAPHORE;
    requestTWI(GYRO_YAW_START, 2);
  }
  if(leftSensor.semaphore & DATA_READY) {
    startIRReading(&rightSensor);
    leftSensor.semaphore &= ~DATA_READY;
    //TODO: read left sensor data
  }
  if(semaphore & TWI_DATA_READY_SEMAPHORE) {
    semaphore &= TWI_DATA_READY_SEMAPHORE;
    //TODO read data using retrieveTWI
  }
  if(rightSensor.semaphore & DATA_READY) {
    rightSensor.semaphore &= ~DATA_READY;
    //TODO: read and store right sensor data
  }
}
