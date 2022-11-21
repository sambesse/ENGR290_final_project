#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "TWI.h"

//sensor header, includes the sampling of all distance sensors and IMU
#define IMU_ADDR 0x68 
#define CONFIG_ADDR 0x1A
#define GYRO_CONFIG_ADDR 0x1B
#define ACCEL_CONFIG_ADDR 0x1C
#define GYRO_OUT_START 0x43
#define GYRO_YAW_START 0x47
#define ACCEL_OUT_START 0x3B
#define GYRO_250 0x00
#define GYRO_500 0x08
#define GYRO_1000 0x10
#define GYRO_2000 0x18
#define GYRO_XST 0x80
#define GYRO_YST 0x40
#define GYRO_ZST 0x20
#define ACCEL_2G 0x00
#define ACCEL_4G 0x08
#define ACCEL_8G 0x10
#define ACCEL_16G 0x18
#define ACCEL_XST 0x80
#define ACCEL_YST 0x40
#define ACCEL_ZST 0x20

#define LEFT_SENSOR_MUX_VAL (0) //TODO make this correct
#define RIGHT_SENSOR_MUX_VAL (1) //TODO make this correct
//constants fron US sensor semaphore
#define IN_PULSE (1 << 7)
#define DATA_READY (1 << 6)

typedef struct {
  uint16_t pulseStart;
  uint16_t pulseEnd;
  uint16_t pulseLength;
  uint8_t semaphore;
} USSensorData;

typedef struct {
  uint8_t adcMux;
  uint16_t adcResult;
  uint8_t semaphore;
} ADCData;

//init function for IMU
void initIMU();

//setups up pointers and starts the conversion
void startIRReading(ADCData* dataPtr); 

//sets up the interrupt and pointers
void initFrontSensor(USSensorData* frontSensorData); 

//initiailze ADC needed for reading IR sensors
void initIRSensors();

#endif
