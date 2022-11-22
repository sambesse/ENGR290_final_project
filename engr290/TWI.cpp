#include "TWI.h"

uint8_t targetAddr;
FIFOBuffer txBuffer;
FIFOBuffer rxBuffer;
uint8_t nextTxByte = 0;
uint8_t nextRxByte = 0;
uint8_t RNW = 0;
uint8_t remainingBytes = 0;
extern uint8_t semaphore;

void produce_byte(FIFOBuffer* fifobuffer, const uint8_t data);
uint8_t consume_byte(FIFOBuffer* fifobuffer);
void printTWIStatus();

ISR(TWI_vect) {
  Serial.println("inside TWI int");
  ///printTWIStatus();
  switch(TWSR) {
    case 0x08:  //start has been transmitted
      TWDR = targetAddr << 1 | 0; //write
      TWCR |= (1 << 7); //clear TWINT
      TWCR &= ~(1 << 5); //clear start
      break;
    case 0x10: //repeated start, only happeens in the reading mode
      TWDR = targetAddr << 1 | 1; //read
      TWCR |= (1 << 7) | (1 << 6); //enable responding with ack
      TWCR &= ~(1 << 5); //clear start
      break;
    case 0x18: // addr + write transmiteed ack recieved
      TWDR = consume_byte(&txBuffer);
      TWCR |= (1 << 7);
      break;
    case 0x20: //add + write transmitted nack recieved
      TWCR |= (1 << 7) | (1 << 5); //reset interrupt and retransmit start
      break;
    case 0x28://data byte transmitted and ack recieved
      if(RNW) {//this means register address sent and ack recieved, now retransmit start
        TWCR |= (1 << 7) | (1 << 5); //clear interrupt and send restart
      } else {
        if(remainingBytes--) {
          TWDR = consume_byte(&txBuffer);
          TWCR |= (1 << 7); //clear interrupt 
        } else {
          TWCR |= (1 << 7) | (1 << 4); // clear interrupt and assert stop
          semaphore &= ~TWI_BUSY_SEMAPHORE;
          Serial.println("not busy"); 
        }
      }
      break;
    case 30: //data byte transmiteed and nack recieved, idk?
      if(--remainingBytes) {
        TWDR = consume_byte(&txBuffer);
        TWCR |= (1 << 7); //clear interrupt 
      } else {
        TWCR |= (1 << 7) | (1 << 4); // clear interrupt and assert stop
      }
      break;
    case 0x38: //arbitration lost or not ack bit
      TWCR |= (1 << 5) | (1 << 7); //set start and interrupt
      break;
    case 0x40: //addr + read transmitted, ack recieved
      if (remainingBytes > 1) {
        TWCR |= (1 << 7) | (1 << 6); //clear interrupt and enable ACK
      } else {
        TWCR |= (1 << 7);
        TWCR &= ~(1 << 6);
      }
      break;
    case 48: // addr + read transmitted, nack recieved
      TWCR |= (1 << 5) | (1 << 7); //set start and interrupt
      break;
    case 0x50: //data byte recieved and ack returned
      produce_byte(&rxBuffer, TWDR);
      if (--remainingBytes > 1) { //TODO: make sure receieved bytes are counted properly
        TWCR |= (1 << 7) | (1 << 6); //clear interrupt and enable ACK
      } else {
        TWCR |= (1 << 7);
        TWCR &= ~(1 << 6);
      }
      break;
    case 0x58: //data  byte reieved and nack returned
      produce_byte(&rxBuffer, TWDR);
      remainingBytes--; //should now be 0
      TWCR |= (1 << 7) | (1 << 4); //clear interrupt and assert stop
      semaphore &= ~TWI_BUSY_SEMAPHORE;
      semaphore |= TWI_DATA_READY_SEMAPHORE; //signal that data is ready
      Serial.println("DATA READY");
      //printTWIStatus();
      break;
  }
}

void initTWI(const uint8_t slaveAddr) {
  while(semaphore & TWI_BUSY_SEMAPHORE);
  targetAddr = slaveAddr;
  //going for bit rate of 400KHz. fastest possible without changing CPU clock is 62.5KHz.
  for(uint8_t i = 0; i < TWI_BUFFER_LENGTH; i++) {
    rxBuffer.buf[i] = 0;
    txBuffer.buf[i] = 0;
  }
  rxBuffer.writePtr = 0;
  txBuffer.writePtr = 0;
  rxBuffer.readPtr = 0;
  rxBuffer.writePtr = 0;
  TWBR = 8; //fastest clock rate
  TWSR = 0; //prescaler = 1
  TWCR = 0x05; //enabled TWI and TWI interrupt
}

void writeTWI(const uint8_t regAddr, const uint8_t data) {
  while(semaphore & TWI_BUSY_SEMAPHORE);
  //Serial.println("inside write function");
  produce_byte(&txBuffer, regAddr);
  //Serial.println("byte produced");
  produce_byte(&txBuffer, data);
  RNW = 0; //set to write mode
  remainingBytes = 2;
  semaphore |= TWI_BUSY_SEMAPHORE;
  TWCR |= (1 << 5); //set START bit
}

void writeTWI(const uint8_t regAddr, const uint8_t* const data, const uint8_t len) {
  while(semaphore & TWI_BUSY_SEMAPHORE);
  produce_byte(&txBuffer, regAddr);
  if(txBuffer.writePtr + len < TWI_BUFFER_LENGTH) {
    memcpy(txBuffer.buf + txBuffer.writePtr, data, len);
    txBuffer.writePtr += len;
    RNW = 0;
    remainingBytes = len; //don't include register address
    semaphore |= TWI_BUSY_SEMAPHORE;
    TWCR |= (1 << 5);
  }
}

void requestTWI(const uint8_t regAddr, const uint8_t len) {
  uint8_t dummy = 0;
  while(semaphore & TWI_BUSY_SEMAPHORE) {
    Serial.println(semaphore, BIN);
  }
  produce_byte(&txBuffer, regAddr);
  RNW = 1; //set to read mode
  semaphore |= TWI_BUSY_SEMAPHORE;
  remainingBytes = len;
  TWCR |= (1 << 5); //start
}

void receiveTWI(uint8_t* data, const uint8_t len) {
  while(!(semaphore & TWI_DATA_READY_SEMAPHORE));
  if (rxBuffer.readPtr + len < TWI_BUFFER_LENGTH) {
    memcpy(rxBuffer.buf + rxBuffer.readPtr, data, len);
    rxBuffer.readPtr += len;
    if (rxBuffer.readPtr == rxBuffer.writePtr) {//if you've caught up reset the buffer
      rxBuffer.readPtr = rxBuffer.writePtr = 0;
    }
  } else {
    //TODO: do something, but things are fucked
  }
}

void receiveTWI(uint16_t* data) {
  receiveTWI((uint8_t*)data, 2);
}

void produce_byte(FIFOBuffer* fifobuffer, const uint8_t data) {
  //Serial.println("producing");
  //delay(50);
  if (fifobuffer->writePtr < TWI_BUFFER_LENGTH-1) {
    //Serial.println("inside if");
    //delay(10);
    fifobuffer->buf[fifobuffer->writePtr++] = data;
    //Serial.println("after buffer write");
    //delay(20);    
  }
  //TODO: do something if buffer overflows
}

uint8_t consume_byte(FIFOBuffer* fifobuffer) {
  uint8_t ret;
  if (fifobuffer->readPtr < TWI_BUFFER_LENGTH-1) {
    ret = fifobuffer->buf[fifobuffer->readPtr++];
  }
  if (fifobuffer->readPtr == fifobuffer->writePtr) {
    fifobuffer->readPtr = fifobuffer->writePtr = 0;
  }
  return ret;
}

void printTWIStatus() {
  Serial.print("TWI Status Register: 0x");
  Serial.println(TWSR, HEX);
  Serial.print("TWI Command Register: 0x");
  Serial.println(TWCR, HEX);
  Serial.print("TWI Data Register: ");
  Serial.println(TWDR);
  Serial.print("bytes remaining to send: ");
  Serial.println(remainingBytes);
  Serial.print("Reading or writing: ");
  if(RNW)
    Serial.println("reading");
  else
    Serial.println("writing");
  Serial.print("contents of transmit buffer: ");
  for(uint8_t i = txBuffer.readPtr; i < txBuffer.writePtr; i++) {
    Serial.print(txBuffer.buf[i]); Serial.print(", ");
  }
  Serial.println();
  Serial.print("contents of receive buffer: ");
  for(uint8_t i = rxBuffer.readPtr; i < rxBuffer.writePtr; i++) {
    Serial.print(rxBuffer.buf[i]); Serial.print(", ");
  }
  Serial.println();
  Serial.print("status of semaphore: ");
  Serial.println(semaphore, BIN);
  Serial.print("tx read pointer: ");
  Serial.println(txBuffer.readPtr);
  Serial.print("tx write pointer: ");
  Serial.println(txBuffer.writePtr);
}
