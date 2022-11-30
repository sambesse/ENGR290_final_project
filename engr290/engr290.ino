#include <Arduino.h>
#include "TWI.h"
#include "sensors.h"
#include "timing.h"
#include "position.h"

#define STOP_DISTANCE (350)
#define SLOW_DISTANCE (700)

#define SLOW_ANGLE (45)
#define STOP_ANGLE (90)

void turnLeft();
void turnRight();
void turnStraight();
void setThrust(uint8_t strength);
void setLift(uint8_t lift);

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
  float absOrientation = 0;
  uint16_t dxf; //rate of change of front sensor
  //float vel; //potentially leave this in there for the velocity gotten from acc integration if we choose to implement that
} PositionData;

enum State {STRAIGHT, TURNING};

State state = STRAIGHT;
USSensorData frontSensor;
SensorData rawData;
PositionData posData;
uint8_t semaphore = 0;
uint8_t cnt = 0;
uint8_t LNR = 0;

void setup() {
  Serial.begin(9600); // comment out for actual run
  delay(50);
  Serial.println("start");
  rawData.leftSensorData = 0;
  rawData.rightSensorData = 0;
  rawData.frontSensorData = 0;
  rawData.yawRate = 0;
  posData.orientation = 0;
  initTiming();
  initIMU();
  delay(75);
  //calibrateGyro();
  initFrontSensor(&frontSensor);
  initPositionModel(posData.orientation);
  setLift(50);
  analogReference(5);
  delay(500);
  turnStraight();
  setThrust(100);
  Serial.println("end");
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
    //Serial.println("inside IMU semaphore loop");
    readRegN(GYRO_YAW_START, 1, &rawData.yawRate);
    //delay(1);
    //Serial.println("after IMU read");
    tickModel(rawData.yawRate);
    if (posData.orientation < 0) {
      posData.absOrientation = -posData.orientation;
    } else {
      posData.absOrientation = posData.orientation;
    }
    semaphore &= ~IMU_SEMAPHORE;
  }
  //*/
  //*
  if(frontSensor.semaphore & DATA_READY) {
    rawData.frontSensorData = frontSensor.pulseLength;
    //Serial.println(rawData.frontSensorData);
    frontSensor.semaphore &= ~DATA_READY;
  }
  //*/
  //*
  //Serial.print("left sensor: "); Serial.println(rawData.leftSensorData);
  //Serial.print("right sensor: "); Serial.println(rawData.rightSensorData); 
  
  Serial.print("front: "); Serial.println(rawData.frontSensorData);
  //Serial.print("State: "); Serial.println(state);
  //*/
  /*
  if(semaphore & CONTROL_SEMAPHORE) {
    Serial.println("inside control");
    if (state == STRAIGHT) {
      if (rawData.frontSensorData < SLOW_DISTANCE && rawData.frontSensorData > STOP_DISTANCE) {
        Serial.println("slowing");
        Serial.print("front: "); Serial.println(rawData.frontSensorData);
        setThrust((rawData.frontSensorData - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE));
      } else if (rawData.frontSensorData <= STOP_DISTANCE ) {
        Serial.println("Stopping");
        Serial.print("front: "); Serial.println(rawData.frontSensorData);
        setThrust(0);
        if (rawData.leftSensorData < rawData.rightSensorData) {
          turnLeft();
          Serial.print("turning left, left val: "); Serial.print(rawData.leftSensorData); Serial.print("right val: "); Serial.println(rawData.rightSensorData);
        } else {
          turnRight();
        }
        
        state = TURNING;
      }
    } else {
      Serial.println("turning");
      Serial.print("yaw rate: "); Serial.println(rawData.yawRate); 
      Serial.print("angle: "); Serial.println(posData.absOrientation);
      if (posData.absOrientation >= SLOW_ANGLE && posData.absOrientation < STOP_ANGLE) {
        setThrust((posData.absOrientation - SLOW_ANGLE) / (SLOW_ANGLE - STOP_ANGLE));
      } else if (posData.absOrientation >= STOP_ANGLE) {
        Serial.println("going back to straight");
        setThrust(0);
        turnStraight();
        state = STRAIGHT;
        resetReference(); 
      } 
    }
    semaphore &= ~CONTROL_SEMAPHORE;
  }
  //*/
}

void turnLeft() {
  OCR0A = SERVO_LEFT;
  setThrust(75);
}

void turnRight() {
  OCR0A = SERVO_RIGHT;
  setThrust(75);
}

void turnStraight() {
  OCR0A = SERVO_MIDDLE;
  setThrust(75);
}

void setThrust(uint8_t strength) {
  OCR0B = (strength) * 255 / 100;
}

void setLift(uint8_t lift) {
  OCR1A = (lift) * 65535 / 100;
}
