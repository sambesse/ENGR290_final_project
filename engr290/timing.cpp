#include "timing.h"
uint16_t count = 0;
extern uint8_t semaphore; 

ISR(TIMER2_OVF_vect) { // overflows every 2040us 
    count++;
    semaphore |= IMU_SEMAPHORE;
    if (count == 5) {
      semaphore |= CONTROL_SEMAPHORE;
    }
    if (count == 20) {
      semaphore |= IR_SEMAPHORE;
      count = 0;
    }
    
    TIFR2 |= 1; //clear overflow flag
}

void initTiming () { //timer 0 is going to be used to set the semaphore. 
  TIMSK2 = 0x01; //enable timer overflow interrupt
  //TCNT2 = TIMER2_CNTIN;
  TCCR2A = 0;
  TCCR2B = 0x01; // no force output compare, prescalar = 1, starts the timer
  DDRD |= (1 << 5) | (1 << 6);
  TCCR0A = 0xA3; // fast pwm mode
  TCCR0B = 0x04; //prescalar = 8, period of 2040us
  OCR0A = SERVO_MIDDLE;// neutral angle
  OCR0B = 250; //full
}
