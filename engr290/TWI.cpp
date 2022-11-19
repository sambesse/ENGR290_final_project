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

void TWI_isr(void) {
  switch(TWSR) {
    case 0x08:  //start has been transmitted
      TWDR = targetAddr << 1 | 0; //write
      TWCR |= (1 << 7); //clear TWINT
      TWCR &= ~(1 << 5); //clear start
      break;
    case 0x10: //repeated start, only happeens in the reading mode
      TWDR = targetAddr << 1 | 1; //read
      TWCR |= (1 << 7);
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
        if(--remainingBytes) {
          TWDR = consume_byte(&txBuffer);
          TWCR |= (1 << 7); //clear interrupt 
        } else {
          TWCR |= (1 << 7) | (1 << 4); // clear interrupt and assert stop
          semaphore &= ~TWI_BUSY_SEMAPHORE; 
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
        TWCR = (1 << 7) | (1 << 6); //clear interrupt and enable ACK
      } else {
        TWCR = (1 << 7);
        TWCR &= ~(1 << 6);
      }
      break;
    case 48: // addr + read transmitted, nack recieved
      TWCR |= (1 << 5) | (1 << 7); //set start and interrupt
      break;
    case 0x50: //data byte recieved and ack returned
      produce_byte(&rxBuffer, TWDR);
      if (--remainingBytes > 1) { //TODO: make sure receieved bytes are counted properly
        TWCR = (1 << 7) | (1 << 6); //clear interrupt and enable ACK
      } else {
        TWCR = (1 << 7);
        TWCR &= ~(1 << 6);
        semaphore |= TWI_DATA_READY_SEMAPHORE; //signal that data is ready
      }
      break;
    case 0x58: //data  byte reieved and nack returned
      produce_byte(&rxBuffer, TWDR);
      remainingBytes--; //should now be 0
      TWCR |= (1 << 7) | (1 << 4); //clear interrupt and assert stop
      semaphore &= ~TWI_BUSY_SEMAPHORE;
      break;
  }
}

void initTWI(const uint8_t slaveAddr) {
  while(semaphore & TWI_BUSY_SEMAPHORE);
  targetAddr = slaveAddr;
  //going for bit rate of 400KHz. fastest possible without changing CPU clock is 62.5KHz.
  TWBR = 0; //fastest clock rate
  TWSR = 0; //prescaler = 1
  TWCR = 0x05; //enabled TWI and TWI interrupt
}

void writeTWI(const uint8_t regAddr, const uint8_t data) {
  while(semaphore & TWI_BUSY_SEMAPHORE);
  produce_byte(&txBuffer, regAddr);
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
    remainingBytes = len + 1;
    semaphore |= TWI_BUSY_SEMAPHORE;
    TWCR |= (1 << 5);
  }
}

void requestTWI(const uint8_t regAddr, const uint8_t len) {
  while(semaphore & TWI_BUSY_SEMAPHORE);
  produce_byte(&txBuffer, regAddr);
  RNW = 1; //set to read mode
  semaphore |= TWI_BUSY_SEMAPHORE;
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

void produce_byte(FIFOBuffer* fifobuffer, const uint8_t data) {
  if (fifobuffer->writePtr < TWI_BUFFER_LENGTH-1) {
    fifobuffer->buf[fifobuffer->writePtr++] = data;
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
