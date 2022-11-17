#ifndef TIMING_H
#define TIMING_H

uint32_t msTimeElapsed;
uint8_t semaphore = 0;

#define IMU_SAMPLE_SEMAPHORE (1 << 7) //period of 125us, I2C clock rate of 400KHz. 
#define IR_SAMPLE_SEMAPHORE (1 << 6) //longest possible measurement period from device is 47ms. 400 longer period than IMU
//the US sensor doesn't require any control signal, it will simply output pulses on the pulse pin continuously

#define TIMER0_CNTIN (255 - 125) //125 counts with a prescalar of 1 gives 125us with a 1MHz io clock

void initTiming();

#endif
