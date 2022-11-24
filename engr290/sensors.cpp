#include "sensors.h"
USSensorData* data; 
ADCData* currentADC;
uint8_t level;

ISR(INT1_vect) {//configured to trigger on every edge of the pulse line
  level = PIND & (1 << 3); //check the pin level
  if(!level) {
    data->pulseEnd = TCNT1;
    if (data->pulseEnd < data->pulseStart) {
      data->pulseLength = (65535 - data->pulseStart) + data->pulseEnd;
    } else {
      data->pulseLength = data->pulseEnd - data->pulseStart;
    }
    data->semaphore |= DATA_READY;
  } else {
    data->pulseStart = TCNT1;
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
  writeReg(CONFIG_ADDR, 0x02); //setup up weakest digital low pass filter
  writeReg(GYRO_CONFIG_ADDR, GYRO_250);
  writeReg(ACCEL_CONFIG_ADDR, ACCEL_2G);
}

void startIRReading(ADCData* dataPtr) {
  currentADC = dataPtr;
//  ADMUX = dataPtr->adcMux;
//  ADCSRA |= (1 << 6);
//  uint8_t i = 0;
//  while(ADCSRA & (1 << 6)) {
//    i++;
//  }
//  Serial.println("ADC conversion complete");
//  Serial.print("ADC MUX: ");
//  Serial.println(ADMUX, HEX);
//  Serial.print("ADC status register: ");
//  Serial.println(ADCSRA, HEX);
//  Serial.print("ADC result: ");
//  Serial.println(ADC);
  dataPtr->adcResult = analogRead(A2);
  dataPtr->semaphore |= DATA_READY;
}

void initFrontSensor(USSensorData* frontSensorData) {
  data = frontSensorData;
  data->semaphore = 0;
  DDRD &= ~(1 << 3);
  EICRA = 0x04; //set interrupt 1 for any logic change 
  EIMSK = 0x02; //enable interrupt 1
  TCCR1A = 0;
  TCCR1B = 3; //set up prescaler of 256 
}

void initIRSensors() {
  DDRC &= ~(1 << 3);
  DDRC &= ~(1 << 2); //set PC2 and PC3 as input
  DIDR0 = 0;
  ADMUX = 0;
  ADCSRA = 0x80; //enable ADC with interrupt, nonblocking
}
