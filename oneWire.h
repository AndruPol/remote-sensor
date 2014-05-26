#ifndef __ONE_WIRE_H__
#define __ONE_WIRE_H__
#include "ch.h"
#include "hal.h"

/*
  This is a library for OneWire Master bus http://en.wikipedia.org/wiki/1-Wire
  based on advices found on dallas documentation : 
  http://www.maximintegrated.com/app-notes/index.mvp/id/214
  this library use an usart to avoid to consume cpu cycle in bit banging

  the library is synchronous, but when waiting it yield the processor to chibios.

  Alexandre Bustico
  alexandre.bustico@enac.fr

  TODO : envoyer les 0xff depuis la flash pour economiser de la stack
	 port discovery
         temperature en asynchrone sur plusieurs capteurs


 */


struct OneWireDriver;
typedef struct OneWireDriver  OneWireDriver;

typedef struct {
  uint8_t addr[8];
} OneWireRomAddress;

typedef struct {
  ioportid_t dqPort; // the port for the data IO
  uint8_t    dqPad;  // pad for the data IO, shloud be the tx of an usart
  uint8_t    dqAlternate; // alternate function, see stm32 ref manuel to find it
  UARTDriver *uartd; // Chibios Uart Driver used as oneWire interface
} OneWireConfig;



/*
  create OneWireDriver object, initialise UART
 */
void     oneWireInit (OneWireDriver* drv, const OneWireConfig *cfg);

/*
  send a reset condition on the OneWire bus to reinitialise all
  slave on this bus
 */
bool_t   oneWireReset (OneWireDriver* drv);


/*
  basic io : send/receive bit, byte, buffers
  since every OneWire Bit involve uart byte,
  oneWireSend need to reserve len*8 bytes on the stack
  oneWireReceive need to reserve len*8*2 bytes on the stack
  So Stack size should be calculated according to this if you send receive
  big bunch of bytes.
 */
bool_t   oneWireReceiveBit (OneWireDriver* drv);
void     oneWireSendBit (OneWireDriver* drv, bool_t bit);
void     oneWireSendByte (OneWireDriver* drv, uint8_t command);
uint8_t  oneWireReceiveByte (OneWireDriver* drv);
bool_t   oneWireSend (OneWireDriver* drv, const uint8_t *command, uint8_t len);
bool_t   oneWireReceive (OneWireDriver* drv, uint8_t *buffer, uint8_t len);

/* 
   get rom address of the slave when there is just ONE slave connected
   otherwise (multiple slaves connected), use oneWireSearchRom
*/
bool_t   oneWireReadRom (OneWireDriver* drv, OneWireRomAddress *addr);


/* 
   get all rom address when multiple slave are connected
   depending on  conditional value, get All the
   slave or just the slave which are in alarm state (cf conditional search)
   it's your responsability to give a pointer on an sufficently large
   array for all the slave, otherwise you will miss some slave.
*/
uint8_t  oneWireSearchRom (OneWireDriver* drv, bool_t conditional, 
			   OneWireRomAddress *addrs, uint8_t len);


/* 
   crc helper function implementing crc8 used bu oneWire slave, to ensure that
   the data received are valid
 */
//uint8_t  oneWireCrc8( uint8_t *addr, uint8_t len);

uint8_t  owCrc8( uint8_t *addr, uint8_t len);

/*
  If more than one thread would communicate with different slaves,
  call to oneWire function should be pritected by these functions
 */
void oneWireAcquireBus (OneWireDriver* drv);
void oneWireRealeaseBus (void);


/*
  This is private data, for internal library state only
 */

// Private Data Daclaration
typedef enum {ModeGpioIn, ModeSerialReset, ModeSerialTransfert} SerialMode;

struct OneWireDriver {
  OneWireConfig config;
  SerialMode currentSm;
  Mutex acquireLock;
  BinarySemaphore semSent;
  BinarySemaphore semReceive;
  uint8_t lastDiscrep ;
  bool_t  doneFlag ; 
};


#endif //__ONE_WIRE_H__
