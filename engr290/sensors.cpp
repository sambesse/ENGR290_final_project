void initIMU() {
  uint8_t configString [3] = {0x01, GYRO_250, ACCEL_2G}; //set DLPF on, gyro to +-250 and accel to +-2g
  initTWI(IMU_ADDR);
  writeTWI(CONFIG_ADDR, configString, 3); //TODO: need to make this function for burst write, already taken care of in ISR
}

void initIRSensors() {
  DDRC &= ~(1 << 3);
  //TODO set other IR pin as input as well
  ADMUX = 0x03;
  ADCSRA = 0x88; //enable ADC with interrupt
}
