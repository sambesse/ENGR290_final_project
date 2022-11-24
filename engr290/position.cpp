#include "position.h"

float* acc;
int16_t prevSample;
uint32_t lastSample = 0;
uint32_t currentSample;
void initPositionModel(float& ori) {
  acc = &ori;
  prevSample = 0;
}

void tickModel(int16_t &latestSample) {
  currentSample = millis();
  if(latestSample - GYRO_BIAS > 50 || latestSample - GYRO_BIAS < -50) { //only take values that differ from the 0 point.
    if (latestSample > prevSample) {
      *acc += ((((latestSample - prevSample) / 2.0) + prevSample) * GYRO_SCALER) * (currentSample - lastSample) / 1000.0;
    } else if (prevSample > latestSample) {
      *acc += ((((prevSample - latestSample) / 2.0) + latestSample) * GYRO_SCALER) * (currentSample - lastSample) / 1000.0;
    }
  }
  lastSample = currentSample; 
}

void resetReference() {
  *acc = 0;
}
