#ifndef POSITION_H
#define POSITION_H

#include "sensors.h"

void initPositionModel(float& ori);

void tickModel(int16_t &latestSample);

void resetReference(); 

void calibrateGyro();

#endif
