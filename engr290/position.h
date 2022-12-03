#ifndef POSITION_H
#define POSITION_H

#include "sensors.h"

#define GYRO_BIAS (-260)

void initPositionModel(float& ori);

void tickModel(int16_t &latestSample);

void resetReference(); 

void calibrateGyro();

#endif
