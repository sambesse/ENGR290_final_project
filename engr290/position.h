#include <Arduino.h>

#define GYRO_SCALER (500.0/65535.0)
#define IMU_DELTA_T 0.000256f
#define GYRO_BIAS (-350)

void initPositionModel(float& ori);

void tickModel(int16_t &latestSample);

void resetReference(); 
