#include "position.h"

float* acc;
int16_t prevSample;
uint32_t lastSample = 0;
uint32_t currentSample;
float gyroScaler = 500.0f / 65535.0f;
float gyroBias = 0;

void initPositionModel(float& ori) {
  acc = &ori;
  prevSample = 0;
}

void tickModel(int16_t &latestSample) {
  currentSample = millis();
  latestSample -= gyroBias;
  if (latestSample > prevSample) {
    *acc += (int)((((latestSample - prevSample) / 2) + prevSample) * gyroScaler) * (float)(currentSample - lastSample) / 1000.0 * 8.0;
  } else if (prevSample > latestSample) {
    *acc += ((((prevSample - latestSample) / 2.0) + latestSample) * gyroScaler) * (currentSample - lastSample) / 1000.0 * 8.0;
  }
  lastSample = currentSample; 
}

void resetReference() {
  *acc = 0;
}

void calibrateGyro() {
  int32_t sum = 0;
  int16_t temp = 0;
  for(uint8_t i = 0; i < 10; i++) {
    readRegN(GYRO_YAW_START, &temp, 1);
    delay(8); //attempt to prevent I2C crashes
    sum += temp;
  }
  gyroBias = sum / 10.0f;
}
