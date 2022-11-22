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

typedef struct {
  float frontDistance; //would rather this be an int, but we'll see
  //uint16_t dxl; //rate of change of left distance sensor
  //uint16_t dxr; //rate of change of right distance sensor
  float orientation = 0; //yaw angle relative to starting point
  uint16_t dxf; //rate of change of front sensor
  //float vel; //potentially leave this in there for the velocity gotten from acc integration if we choose to implement that
} PositionData;

enum State {STRAIGHT, TURNING};

State state = STRAIGHT;
USSensorData frontSensor;
ADCData leftSensor;
ADCData rightSensor;
SensorData rawData;
uint8_t semaphore = 0;

void setup() {
  Serial.begin(9600); // comment out for actual run
  delay(50);
  Serial.println("start");
  rawData.leftSensorData = 0;
  rawData.rightSensorData = 0;
  rawData.frontSensorData = 0;
  rawData.yawRate = 0; 
  leftSensor.adcMux = LEFT_SENSOR_MUX_VAL;
  rightSensor.adcMux = RIGHT_SENSOR_MUX_VAL;
  initTiming();
  //initIMU();
  //initIRSensors();
  initFrontSensor(&frontSensor);
}

void loop() {
  /*
  if(semaphore & IR_SEMAPHORE) {
    Serial.println("inside ir semaphore");
    startIRReading(&leftSensor);
    semaphore &= ~IR_SEMAPHORE; 
  }
  //*/
  /*
  if(semaphore & IMU_SEMAPHORE) {
    Serial.println("inside IMU semaphore loop");
    requestTWI(GYRO_YAW_START, 2);
    semaphore &= ~IMU_SEMAPHORE;
  }
  //*/
  /*
  if(leftSensor.semaphore & DATA_READY) {
    Serial.println("left sensor data ready");
    Serial.println(ADC);
    startIRReading(&rightSensor);
    rawData.leftSensorData = leftSensor.adcResult;
    leftSensor.semaphore &= ~DATA_READY;
  }
  //*/
  /*
  if(semaphore & TWI_DATA_READY_SEMAPHORE) {
    receiveTWI(&rawData.yawRate);
    Serial.print("received yaw rate: ");
    Serial.println(rawData.yawRate);
    semaphore &= ~TWI_DATA_READY_SEMAPHORE;
  }
  //*/
  /*
  if(rightSensor.semaphore & DATA_READY) {
    Serial.println("right sensor data ready");
    Serial.println(ADC);
    rawData.rightSensorData = rightSensor.adcResult;
    rightSensor.semaphore &= ~DATA_READY;
  }
  //*/
  //*
  if(frontSensor.semaphore & DATA_READY) {
    rawData.frontSensorData = frontSensor.pulseLength;
    Serial.println(rawData.frontSensorData);
    frontSensor.semaphore &= ~DATA_READY;
  }
  //*/
  /*
  Serial.print("left sensor: "); delay(50); Serial.println(rawData.leftSensorData); delay(50); 
  Serial.print("right sensor: "); delay(50); Serial.println(rawData.rightSensorData); delay(50); 
  //Serial.print("front sensor: "); delay(50); Serial.println(rawData.frontSensorData); delay(50); 
  //Serial.print("yaw rate: "); delay(50); Serial.println(rawData.yawRate); delay(50); 
  //*/
}
