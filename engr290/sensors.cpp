#include "sensors.h"
USSensorData* sensorData; 
ADCData* currentADC;

void int1_isr(void) {//configured to trigger on every edge of the pulse line
  if(data->semaphore & IN_PULSE) {
    pulseEnd = TCNT1;
    data->semaphore &= ~IN_PULSE;
    if (pulseEnd < pulseStart) {
      pulseLength = (65535 - pulseStart) + pulseEnd;
    } else {
      pulseLength = pulseStart - pulseEnd; 
    }
    data->semaphore |= DATA_READY;
  } else {
    pulseStart = TCNT1;
    data->semaphore |= IN_PULSE;
  }
  EIFR |= 1 << 1; // clear interrupt 1 flag
}

void adc_isr(void) {
  currentADC->adcResult = ADC;
  currentADC->semaphore |= DATA_READY;
  ADCSCRA |= (1 << 4); //clear interrupt
}

void initIMU() {
  uint8_t configString [3] = {0x01, GYRO_250, ACCEL_2G}; //set DLPF on, gyro to +-250 and accel to +-2g
  initTWI(IMU_ADDR);
  writeTWI(CONFIG_ADDR, configString, 3); //TODO: need to make this function for burst write, already taken care of in ISR
}

void startIRReading(ADCData* dataPtr) {
  currentADC = dataPtr;
  ADMUX = dataPtr->adcMux; 
  ADCSRA |= (1 << 7);
}

void initFrontSensor(USSensorData* frontSensorData) {
  data = frontSensorData;
  data->semaphore = 0;
  EICRA = 0x04; //set interrupt 1 for any logic change 
  EIMSK = 0x02; //enable interrupt 1
}

void initIRSensors() {
  DDRC &= ~(1 << 3);
  //TODO set other IR pin as input as well
  ADCSRA = 0x88; //enable ADC without interrupt, maybe make nonblocking in the future
}
