#ifndef __DS_1820_B_H__
#define __DS_1820_B_H__

#include "oneWire.h"


/* 
   initialise and configure sensors 
 */
void     ds1820BInit (OneWireDriver* drv, const OneWireRomAddress *address, 
		      const uint8_t precBits);


/*
  ask conversion, wait for the conversion to be done, then return value,
  could be the simplest way to acquire data when there is only one sensor
*/
float    ds1820BGetTemp (OneWireDriver* drv, const OneWireRomAddress *address);


/*
  separate ask conversion command and get values.
  this is the fastest way to acquire data when there is a lot of sensors :
  1/ ask conversion for all sensors
  2/ wait time accordingly to precision (see datasheet)
  3/ get the temperature for all sensors
 */
void     ds1820BAskTemp (OneWireDriver* drv, const OneWireRomAddress *address);
float    ds1820BGGetTempFromRam (OneWireDriver* drv, const OneWireRomAddress *address);

#endif //__DS_1820_B_H__
