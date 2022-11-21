#ifndef TWI_H
#define TWI_H

#define TWI_BUFFER_LENGTH (10)
#include <Arduino.h>
#include "timing.h"



typedef struct {
  uint8_t buf [TWI_BUFFER_LENGTH];
  uint8_t readPtr = 0;
  uint8_t writePtr = 0;
} FIFOBuffer;

void initTWI(const uint8_t slaveAddr);

void writeTWI(const uint8_t regAddr, const uint8_t data);

void writeTWI(const uint8_t regAddr, const uint8_t* const data, const uint8_t len);

void requestTWI(const uint8_t regAddr, const uint8_t len);

void receiveTWI(uint8_t* data, const uint8_t len);

void receiveTWI(uint16_t* data);

#endif
