#include "ch.h"
#include "hal.h"
#include "oneWire.h"
#include "ds1820b.h"
#include "math.h"


void   ds1820BInit (OneWireDriver* drv, const OneWireRomAddress *address, 
		      const uint8_t precBits)
{
  uint8_t initVal[] = {0x4e, 0x00, 0xff, 0x1f};
    
  if (precBits <= 9) {
    initVal[3] |= 0;
  } else if (precBits == 10) {
    initVal[3] |= 0b00100000;
  } else if (precBits == 11) {
    initVal[3] |= 0b01000000;
  } else { // prec bit >= 12
    initVal[3] |= 0b01100000;
  }
  
  oneWireReset (drv);
  if (address == NULL) {
    oneWireSendByte (drv, 0xcc);
  } else {
    oneWireSendByte (drv, 0x55);
    oneWireSend (drv, address->addr, 8); 
  }
  
  oneWireSend (drv, initVal, sizeof(initVal));
}

float  ds1820BGetTemp (OneWireDriver* drv, const OneWireRomAddress *address)
{
  uint8_t ram[9];

  oneWireAcquireBus (drv);
  oneWireReset (drv);
  if (address == NULL) {
    oneWireSendByte (drv, 0xcc);
  } else {
    oneWireSendByte (drv, 0x55);
    oneWireSend (drv, address->addr, 8); 
  }

  oneWireSendByte (drv, 0x44);
  chThdSleepMilliseconds(100); // minimum conv time when 9 bit
  while (oneWireReceiveBit(drv) == FALSE) {
    chThdSleepMilliseconds(10); // waint until conversion is done
  }
  
  oneWireReset (drv);
  if (address == NULL) {
    oneWireSendByte (drv, 0xcc);
  } else {
    oneWireSendByte (drv, 0x55);
    oneWireSend (drv, address->addr, 8); 
  }

  oneWireSendByte (drv, 0xbe);
  oneWireReceive (drv, ram, sizeof(ram));
//  uint8_t crc = oneWireCrc8(ram, 8);
  uint8_t crc = owCrc8(ram, 8);
  if (crc != ram[8]) {
    oneWireRealeaseBus ();
    return (nanf(""));
  } 
  

  const int16_t rawTmp = (ram[1] << 8) | (ram [0]);
  oneWireRealeaseBus ();
  return (rawTmp * 0.0625f);
}


void ds1820BAskTemp (OneWireDriver* drv, const OneWireRomAddress *address)
{
  oneWireAcquireBus (drv);
  oneWireReset (drv);
  if (address == NULL) {
    oneWireSendByte (drv, 0xcc);
  } else {
    oneWireSendByte (drv, 0x55);
    oneWireSend (drv, address->addr, 8); 
  }
  oneWireSendByte (drv, 0x44);
  oneWireRealeaseBus ();
}


float  ds1820BGGetTempFromRam (OneWireDriver* drv, const OneWireRomAddress *address)
{
  uint8_t ram[9];

  oneWireAcquireBus (drv);
 
  oneWireReset (drv);
  if (address == NULL) {
    oneWireSendByte (drv, 0xcc);
  } else {
    oneWireSendByte (drv, 0x55);
    oneWireSend (drv, address->addr, 8); 
  }

  oneWireSendByte (drv, 0xbe);
  oneWireReceive (drv, ram, sizeof(ram));
//  uint8_t crc = oneWireCrc8(ram, 8);
  uint8_t crc = owCrc8(ram, 8);
  if (crc != ram[8]) { 
    oneWireRealeaseBus ();
    return (nanf(""));
  } 

  oneWireRealeaseBus ();
  const int16_t rawTmp = (ram[1] << 8) | (ram [0]);
  return (rawTmp * 0.0625f);
}

