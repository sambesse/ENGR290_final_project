#ifndef TIMING_H
#define TIMING_H
#include <Arduino.h>

#define IMU_SEMAPHORE (1 << 7) //period of 125us, I2C clock rate of 400KHz. 
#define IR_SEMAPHORE (1 << 6) //longest possible measurement period from device is 47ms. 400 longer period than IMU
#define TWI_DATA_READY_SEMAPHORE  (1 << 5)
#define TWI_BUSY_SEMAPHORE (1 << 4)
//the US sensor doesn't require any control signal, it will simply output pulses on the pulse pin continuously

#define TIMER0_CNTIN (255 - 125) //125 counts with a prescalar of 1 gives 125us with a 1MHz io clock

void initTiming();

#endif
