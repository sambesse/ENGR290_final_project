#include <Arduino.h>
#include "TWI.h"
#include "sensors.h"
#include "timing.h"
#include "position.h"

#define STOP_DISTANCE (14000)
#define SLOW_DISTANCE (120)

#define SLOW_ANGLE (20)
#define STOP_ANGLE (40)

#define LIFT_ACTIVE (75)
#define LIFT_IDLE (10)

#define THRUST_STRAIGHT (50)
#define THRUST_TURNING (30)

#define TOUCHING (500)

void slightLeft();
void slightRight();
void turnLeft();
void turnRight();
void turnStraight();
void setThrust(uint8_t strength);
void setLift(uint8_t lift);

typedef struct {
  uint16_t leftSensorData = 0;
  uint16_t rightSensorData = 0;
  uint32_t frontSensorData = 100;
  int16_t yawRate = 0;
} SensorData;

typedef struct {
  float frontDistance; //would rather this be an int, but we'll see
  int16_t dl; //rate of change of left distance sensor
  int16_t dr; //rate of change of right distance sensor
  float orientation = 0; //yaw angle relative to starting point
  float absOrientation = 0;
  int16_t df; //rate of change of front sensor
  //float vel; //potentially leave this in there for the velocity gotten from acc integration if we choose to implement that
} PositionData;

enum State {STRAIGHT, TURNING, STARTUP};

State state = STARTUP;
USSensorData frontSensor;
SensorData rawData;
PositionData posData;
volatile uint8_t semaphore = 0;
volatile uint8_t timer2cnt = 0;
uint8_t cnt = 0;
uint8_t LNR = 0;
uint32_t frontBuf[4] = {0, 0, 0, 0};

void setup() {
  Serial.begin(9600); // comment out for actual run
  delay(50);
  Serial.println("start");
  rawData.leftSensorData = 0;
  rawData.rightSensorData = 0;
  rawData.frontSensorData = 100;
  rawData.yawRate = 0;
  posData.orientation = 0;
  initTiming();
  initIMU();
  delay(75);
  //calibrateGyro();
  initFrontSensor(&frontSensor);
  initPositionModel(posData.orientation);
  setLift(LIFT_IDLE);
  analogReference(5);
  delay(500);
  setLift(LIFT_IDLE*2);
  //turnStraight();
  OCR0A = SERVO_START;
  setThrust(THRUST_STRAIGHT);
  delay(200);
  setLift(LIFT_ACTIVE);
  Serial.println("end");
}

void loop() {
  //*
  if(semaphore & IR_SEMAPHORE) {
    uint16_t ltemp = rawData.leftSensorData;
    uint16_t rtemp = rawData.rightSensorData;
    rawData.leftSensorData = analogRead(LEFT_IR_PIN);
    rawData.rightSensorData = analogRead(RIGHT_IR_PIN);
    posData.dl = ltemp - rawData.leftSensorData;
    posData.dr = rtemp - rawData.rightSensorData;
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
    uint8_t stopping = 0;
    frontBuf[cnt%4] = frontSensor.pulseLength;
    if (frontBuf[cnt%4] > STOP_DISTANCE) {
      posData.df = rawData.frontSensorData - frontBuf[cnt%4];
      rawData.frontSensorData = frontBuf[cnt%4];
    } else {
      for(uint8_t i = 0; i < 4; i++) {
        if(frontBuf[i] <= STOP_DISTANCE) {
          stopping++;
        } else {
          posData.df = rawData.frontSensorData - frontBuf[i];
          rawData.frontSensorData = frontBuf[i];
        }
      }
      if(stopping >= 4) {
        posData.df = rawData.frontSensorData - frontBuf[cnt%4];
        rawData.frontSensorData = frontBuf[cnt%4];
      }
    }
    //Serial.println(rawData.frontSensorData);
    cnt++;
    frontSensor.semaphore &= ~DATA_READY;
  }
  //*/
  //*
  Serial.print("left sensor: "); Serial.println(rawData.leftSensorData);
  Serial.print("right sensor: "); Serial.println(rawData.rightSensorData); 
  
  Serial.print("front: "); Serial.println(rawData.frontSensorData);
  //Serial.print("State: "); Serial.println(state);
  //*/
  //*
  if(semaphore & CONTROL_SEMAPHORE) {
    //Serial.println("inside control");
    if (state == STRAIGHT) {
      setThrust(THRUST_STRAIGHT);
      if (rawData.frontSensorData < SLOW_DISTANCE && rawData.frontSensorData > STOP_DISTANCE) {
        Serial.println("slowing");
        Serial.print("front: "); Serial.println(rawData.frontSensorData);
        setThrust((rawData.frontSensorData - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE));
      } else if (rawData.frontSensorData <= STOP_DISTANCE ) {
        Serial.println("Stopping");
        Serial.print("front: "); Serial.println(rawData.frontSensorData);
        setLift(LIFT_IDLE);
        setThrust(0);
        if (posData.orientation > 70) {
          turnLeft();
          resetReference();
        } else if (posData.orientation < -70) {
          turnRight();
          resetReference();
        }
        delay(300);
        if (rawData.leftSensorData < rawData.rightSensorData) {
          turnLeft();
          Serial.print("turning left, left val: "); Serial.print(rawData.leftSensorData); Serial.print("right val: "); Serial.println(rawData.rightSensorData);
        } else {
          turnRight();
        }
        state = TURNING;
        resetReference();
      } else {
        if (rawData.leftSensorData >= TOUCHING) {
          //Serial.println("left touching");
          //slightRight();
          setLift(100);
          setThrust(100);
        }
        if (rawData.rightSensorData >= TOUCHING) {
          //Serial.println("right touching");
          //slightLeft();
          setLift(100);
          setThrust(100);
        }
      }
    } else if (state == TURNING) {
      Serial.println("turning");
      //Serial.print("yaw rate: "); Serial.println(rawData.yawRate); 
      Serial.print("angle: "); Serial.println(posData.absOrientation);
      //delay(200);
      setLift(LIFT_ACTIVE);
      setThrust(THRUST_TURNING);
      if (posData.absOrientation >= STOP_ANGLE) {
        //Serial.println("going back to straight");
        setThrust(0);
        setLift(LIFT_IDLE);
        turnStraight();
        state = STRAIGHT;
        resetReference();
        delay(200);
        setLift(LIFT_ACTIVE);
        setThrust(THRUST_STRAIGHT);
      } 
    } else if (state == STARTUP) {
      if (cnt >= 20) {
        state = STRAIGHT;
        turnStraight();
      }
    }
    semaphore &= ~CONTROL_SEMAPHORE;
  }
  //*/
}

void slightLeft() {
  OCR0A--;
}

void slightRight() {
  OCR0A++; 
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
