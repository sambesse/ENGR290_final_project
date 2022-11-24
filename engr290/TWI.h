#ifndef TWI_H
#define TWI_H

#define TWI_BUFFER_LENGTH (10)
#include <Arduino.h>
#include "timing.h"
#include <Wire.h>

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
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

void initTWI();

void writeReg(uint8_t addr, uint8_t data);

void readRegN(uint8_t addr, uint8_t bytes, uint16_t* data);

#endif
