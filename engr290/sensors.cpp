#include "sensors.h"

USSensorData* data; 
ADCData* currentADC;
uint8_t level;
extern uint8_t timer2cnt; 

ISR(INT1_vect) {//configured to trigger on every edge of the pulse line
  level = PIND & (1 << 3); //check the pin level
  if(!level) {
    data->pulseEnd = TCNT2;
    if (timer2cnt == 1) {
      data->pulseLength = (255 - data->pulseStart) + data->pulseEnd;
    } else if (timer2cnt == 0) {
      data->pulseLength = data->pulseEnd - data->pulseStart;
    } else {
      data->pulseLength = (255 - data->pulseStart) + data->pulseEnd + 255 * timer2cnt;
    }
    timer2cnt = 0;
    data->semaphore |= DATA_READY;
  } else {
    data->pulseStart = TCNT2;
    timer2cnt = 0;
  }
  EIFR |= 1 << 1; // clear interrupt 1 flag
}

ISR(ADC_vect) {
  currentADC->adcResult = ADC;
  currentADC->semaphore |= DATA_READY;
  ADCSRA |= (1 << 4); //clear interrupt
}

void initIMU() {
  initTWI();
  writeReg(PWR_MGMT_1, 0x0B); //disable temp and set clock to yaw gyro
  writeReg(PWR_MGMT_2, 0x3E); //disables everything but the yaw gyro
  writeReg(CONFIG_ADDR, 0x01); //setup up weakest digital low pass filter
  writeReg(GYRO_CONFIG_ADDR, GYRO_250); 
}

void initFrontSensor(USSensorData* frontSensorData) {
  data = frontSensorData;
  data->semaphore = 0;
  DDRD &= ~(1 << 3);
  EICRA = 0x04; //set interrupt 1 for any logic change 
  EIMSK = 0x02; //enable interrupt 1
  TCCR1A = 0x82; 
  TCCR1B = 3; //set up prescaler of 256 
  OCR1A = 0;
  DDRB |= (1 << 1);
}
