#include <Arduino.h>
#include "TWI.h"
#include "sensors.h"
#include "timing.h"

void turnLeft();
void turnRight();
void turnStraight();

typedef struct {
  uint16_t leftSensorData = 0;
  uint16_t rightSensorData = 0;
  uint16_t frontSensorData = 0;
  int16_t yawRate = 0;
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
SensorData rawData;
uint8_t semaphore = 0;
uint8_t cnt = 0;
uint8_t LNR = 0;

void setup() {
  Serial.begin(9600); // comment out for actual run
  //delay(50);
  Serial.println("start");
  rawData.leftSensorData = 0;
  rawData.rightSensorData = 0;
  rawData.frontSensorData = 0;
  rawData.yawRate = 0;
  initTiming();
  initIMU();
  //initFrontSensor(&frontSensor);                                                                                                                                                                                                                                                                                 digitalWrite(4, HIGH);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH); //turn on lift fan
  analogReference(5);
  turnMiddle();
  delay(500);
  turnLeft();

}

void loop() {
  //*
  if(semaphore & IR_SEMAPHORE) {
    rawData.leftSensorData = analogRead(LEFT_IR_PIN);
    rawData.rightSensorData = analogRead(RIGHT_IR_PIN);
    semaphore &= ~IR_SEMAPHORE;
  }
  //*/
  //*
  if(semaphore & IMU_SEMAPHORE) {
    Serial.println("inside IMU semaphore loop");
    readRegN(GYRO_YAW_START, 2, &rawData.yawRate);
    semaphore &= ~IMU_SEMAPHORE;
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
  if(frontSensor.semaphore & DATA_READY) {
    rawData.frontSensorData = frontSensor.pulseLength;
    Serial.println(rawData.frontSensorData);
    frontSensor.semaphore &= ~DATA_READY;
  }
  //*/
  /*
  if(semaphore & CONTROL_SEMAPHORE) {
    if (state == STRAIGHT) {
      if (rawData.frontSensorData < 400) {
        if (rawData.leftSensorData < rawData.rightSensorData) {
          turnLeft();
        } else {
          turnRight();
        }
        state = TURNING;
      }
    } else if (rawData.yawRate > 80 && rawData.yawRate < 100) {
      turnStraight();
      state = STRAIGHT; 
    }
    semaphore &= ~CONTROL_SEMAPHORE
  }
  //*/
  //*
  Serial.print("left sensor: "); Serial.println(rawData.leftSensorData);
  Serial.print("right sensor: "); Serial.println(rawData.rightSensorData); 
  
  //Serial.print("front sensor: "); delay(50); Serial.println(rawData.frontSensorData); delay(50); 
  Serial.print("yaw rate: "); Serial.println(rawData.yawRate); 
  //*/
  /*
  if(semaphore & CONTROL_SEMAPHORE) {
    if (state == STRAIGHT) {
      if (rawData.frontSensorData < 400) {
        if (rawData.leftSensorData < rawData.rightSensorData) {
          turnLeft();
        } else {
          turnRight();
        }
        state = TURNING;
      }
    } else if (cnt++ >= 100) {
      turnStraight();
      cnt = 0;
      state = STRAIGHT; 
    }
    semaphore &= ~CONTROL_SEMAPHORE;
  }
  //*/
  //turnRight();
}

void turnLeft() {
  OCR0A = SERVO_LEFT;
}

void turnRight() {
  OCR0A = SERVO_RIGHT;
}

void turnMiddle() {
  OCR0A = SERVO_MIDDLE;
}
