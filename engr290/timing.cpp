#include "timing.h"
uint16_t count =0;
extern uint8_t semaphore; 

ISR(TIMER2_OVF_vect) { // this may be too fast, because the procesor runs so slowly. 
    count++;
    semaphore |= IMU_SEMAPHORE;
    if (count >= 400) {
      semaphore |= IR_SEMAPHORE;
      count = 0;
    }
    TIFR2 |= 1; //clear overflow flag
}

void initTiming () { //timer 0 is going to be used to set the semaphore. 
  TIMSK2 = 0x01; //enable timer overflow interrupt
  TCNT2 = TIMER2_CNTIN;
  TCCR2A = 0;
  TCCR2B = 0x01; // no force output compare, prescalar = 1, starts the timer
}
