#include "timing.h"

void timer0_isr(void) {
  count++;
  semaphore |= IMU_SEMAPHORE;
  if (count >= 400) {
    semaphore |= IR_SEMAPHORE;
    count = 0;
  }
}

void initTiming () { //timer 0 is going to be used to set the semaphore. 
  TIMSK0 = 0x01; //enable timer overflow interrupt
  TCNT0 = TIMER0_CNTIN;
  TCCR0A = 0;
  TCCR0B = 0x01; // no force output compare, prescalar = 1, starts the timer
}
