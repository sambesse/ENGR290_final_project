#include "TWI.h"

void initTWI() {
  Wire.begin(); //starting IMU transaction with read enabled, need to specify pins maybe
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void writeReg(uint8_t addr, uint8_t data) { //not sure how to deal with ACKs
  Wire.beginTransmission(IMU_ADDR); //begin write transaction, addr with a 1 at the end
  Wire.write(addr);
  Wire.endTransmission();
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(data);
  Wire.endTransmission(true);
}

void readRegN(uint8_t addr, uint8_t bytes, uint16_t* data) {
  uint8_t temp0 = 0;
  uint8_t temp1 = 0;
  uint8_t i = 0;
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(addr);
  Wire.endTransmission();
  
  Wire.beginTransmission(IMU_ADDR);
  Wire.requestFrom(IMU_ADDR, bytes);
  delay(50); //50ms plenty of time to wait for a response from IMU
  //Serial.print("inside read reg function, available bytes: "); Serial.println(Wire.available());
  while(Wire.available()) {
    temp0 = Wire.read();
    temp1 = Wire.read();
    data[i] = temp0 << 8 | temp1;
    i += 1;
    //Serial.print("temp 0: "); Serial.println(temp0);
    //Serial.print("temp 1: "); Serial.println(temp1);
  }
  Wire.endTransmission();
}
