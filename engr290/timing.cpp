#include "timing.h"
uint16_t count =0;
void timer0_isr(void) { // this may be too fast, because the procesor runs so slowly. 
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
//  NVIC_ENABLE_IRQ(TIMER0_OVF_ISR);
}
