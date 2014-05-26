
#include "ch.h"
#include "hal.h"
#include "oneWire.h"
#include "string.h"

#define OW_TIMEOUT_MS	10	// break timeout

static struct UsartDictionnary {
  OneWireDriver *drv[1];
  UARTDriver   *uart[1];
} usartDict = { .drv = {NULL},
	       	    .uart = {NULL}
				};



static void oneWireSerialMode (OneWireDriver* drv, SerialMode sm);
static uint8_t oneWireUnpack (uint8_t buffer[8]);
static void oneWirePack (uint8_t cmd, uint8_t buffer[8]);
static uint8_t oneWireNext(OneWireDriver* drv, uint8_t *romBytes, 
			     bool_t conditional);
static uint8_t  oneWireFirst(OneWireDriver* drv, uint8_t *romBytes, 
			     bool_t conditional);
static void usartDictionnaryAddPair (OneWireDriver* drv, UARTDriver *uart);
static OneWireDriver* usartDictionnaryGetDriverByUart (UARTDriver *uart);


// USART CALLBACK
static void txBufferEmpty(UARTDriver *uartp) ;
static void rxErr(UARTDriver *uartp, uartflags_t e);
static void rxEnd(UARTDriver *uartp);
//static void rxChar(UARTDriver *uartp, uint16_t c);



static const UARTConfig uartCfgReset = {
  NULL,
  txBufferEmpty,
  rxEnd,
  NULL, //  rxChar,
  rxErr,
  9600,
  0,
  0,
  USART_CR3_HDSEL  // half duplex mode
};

static const UARTConfig uartCfgTransfert = {
  NULL,
  txBufferEmpty,
  rxEnd,
  NULL, //rxChar,
  rxErr,
  115200,
  0,
  0,
  USART_CR3_HDSEL // half duplex mode
};

static const uint8_t dscrc_table[] = {
  0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
  157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
  35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
  190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
  70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
  219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
  101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
  248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
  140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
  17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
  175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
  50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
  202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
  87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
  233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
  116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};


static const uint8_t owBufferWrite[] = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,0xff, 0xff, 
  0xff, 0xff 
};


void oneWireInit (OneWireDriver* drv, const OneWireConfig *cfg)
{
  memcpy (&(drv->config), cfg, sizeof (OneWireConfig));
  oneWireSerialMode (drv, ModeGpioIn);
  chBSemInit (&(drv->semSent), TRUE);
  chBSemInit (&(drv->semReceive), TRUE);
  chMtxInit (&(drv->acquireLock));
  drv->lastDiscrep = 0;
  drv->doneFlag = FALSE;
  usartDictionnaryAddPair (drv, cfg->uartd);
}

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0])) 


bool_t oneWireReset (OneWireDriver* drv)
{
  uint8_t owPresence;
  const uint8_t resetByte = 0xf0;
  
  oneWireSerialMode (drv, ModeSerialReset);
  
  chBSemReset(&(drv->semSent), TRUE);
  chBSemReset(&(drv->semReceive), TRUE);
  uartStartReceive (drv->config.uartd, 1 , (void *) &owPresence);	
  uartStartSend (drv->config.uartd, 1, &resetByte);
  if (chBSemWaitTimeout(&(drv->semSent), MS2ST(OW_TIMEOUT_MS)) == RDY_TIMEOUT){
	  return FALSE;
  }
  if (chBSemWaitTimeout(&(drv->semReceive), MS2ST(OW_TIMEOUT_MS)) == RDY_TIMEOUT){
	  return FALSE;
  }
  
  oneWireSerialMode (drv, ModeSerialTransfert);
  
  return (owPresence != resetByte);
}


void oneWireSendByte (OneWireDriver* drv, uint8_t command)
{
  oneWireSend (drv, &command, 1);
}

uint8_t oneWireReceiveByte (OneWireDriver* drv)
{
  uint8_t buf;
  oneWireReceive (drv, &buf, 1);

  return buf;
}


void oneWireSendBit (OneWireDriver* drv, bool_t bit)
{
  uint8_t bitBuf;
  bitBuf = bit ? 0xff : 0x00;
  
  chBSemReset(&(drv->semSent), TRUE);
  uartStartSend (drv->config.uartd, 1, &bitBuf);
  chBSemWait (&(drv->semSent));
  
}



bool_t oneWireSend (OneWireDriver* drv, const uint8_t *command, uint8_t len)
{
  const uint32_t owLen = len*8;
  uint8_t cmdBuf[owLen];

  for (uint32_t i=0;  i<len; i++) {
    oneWirePack (command[i], &(cmdBuf[i*8]));
  }
  
  chBSemReset(&(drv->semSent), TRUE);
  uartStartSend (drv->config.uartd, owLen, &cmdBuf);
  if (chBSemWaitTimeout(&(drv->semSent), MS2ST(OW_TIMEOUT_MS)) == RDY_TIMEOUT){
	  return FALSE;
  }
  
  return TRUE;
}

bool_t oneWireReceive (OneWireDriver* drv, uint8_t *buffer, uint8_t len)
{
  const uint32_t owLen = len*8;
  uint8_t owBufferRead[owLen];

  if (owLen > sizeof (owBufferWrite)) {
    return FALSE;
  }
  chBSemReset(&(drv->semReceive), TRUE);
  uartStartReceive (drv->config.uartd, owLen , (void *) owBufferRead);
  chBSemReset(&(drv->semSent), TRUE);
  uartStartSend (drv->config.uartd, owLen, (void *) owBufferWrite);
  if (chBSemWaitTimeout(&(drv->semSent), MS2ST(OW_TIMEOUT_MS)) == RDY_TIMEOUT){
	  return FALSE;
  }
  if (chBSemWaitTimeout(&(drv->semReceive), MS2ST(OW_TIMEOUT_MS)) == RDY_TIMEOUT){
	  return FALSE;
  }
  
  for (uint32_t i=0;  i<len; i++) {
    buffer[i] = oneWireUnpack (&(owBufferRead[i*8]));
  }

  return TRUE;
}


bool_t oneWireReceiveBit (OneWireDriver* drv)
{
  uint8_t rec;
  
  chBSemReset(&(drv->semReceive), TRUE);
  uartStartReceive (drv->config.uartd, 1 , (void *) &rec);
  chBSemReset(&(drv->semSent), TRUE);
  uartStartSend (drv->config.uartd, 1, (void *) owBufferWrite);
  if (chBSemWaitTimeout(&(drv->semSent), MS2ST(OW_TIMEOUT_MS)) == RDY_TIMEOUT){
	  return FALSE;
  }
  if (chBSemWaitTimeout(&(drv->semReceive), MS2ST(OW_TIMEOUT_MS)) == RDY_TIMEOUT){
	  return FALSE;
  }
  
  return (rec == 0xff);
}

bool_t oneWireReadRom (OneWireDriver* drv, OneWireRomAddress *rom)
{
  oneWireReset (drv);
  oneWireSendByte (drv, 0x33);
  oneWireReceive (drv, rom->addr, sizeof (rom->addr));
//  uint8_t crc = oneWireCrc8(rom->addr, 7);
  uint8_t crc = owCrc8(rom->addr, 7);
  if (crc != rom->addr[7]) {
    return FALSE;
  } else {
    return TRUE;
  }
}




uint8_t oneWireSearchRom (OneWireDriver* drv, bool_t conditional,
			  OneWireRomAddress *addrs, uint8_t len)
{
   uint8_t m, cont = 0;
   uint8_t romBytes[8] = {0,0,0,0,0,0,0,0};

   oneWireAcquireBus (drv);
   if(oneWireReset(drv))  {
     if(oneWireFirst(drv, romBytes, conditional)) {    
       // Begins when at least one part found
       uint8_t numROMs = 0;
       do  {
    	   cont++;
    	   for (m=0;m<8;m++) {
    		   addrs[numROMs].addr[m] = romBytes[m];
    	   }
    	   numROMs++; // Continues until no additional
       } while ((numROMs < len) && oneWireNext(drv, romBytes, conditional));   

       while (numROMs < len) {
    	   for (m=0;m<8;m++) {
    		   addrs[numROMs].addr[m] = 0;
    	   }
    	   numROMs++;
       } 
     }
   }
   oneWireRealeaseBus ();
   return cont;   
}


/*uint8_t oneWireCrc8( uint8_t *addr, uint8_t len)
{
   uint8_t crc = 0;
  
  while (len--) {
    crc = dscrc_table[crc ^ *addr++];
  }
  return crc;
}*/

uint8_t crcDS(uint8_t inp, uint8_t crc) {
	inp ^= crc;
	crc = 0;
	if(inp & 0x1)   crc ^= 0x5e;
	if(inp & 0x2)   crc ^= 0xbc;
	if(inp & 0x4)   crc ^= 0x61;
	if(inp & 0x8)   crc ^= 0xc2;
	if(inp & 0x10)  crc ^= 0x9d;
	if(inp & 0x20)  crc ^= 0x23;
	if(inp & 0x40)  crc ^= 0x46;
	if(inp & 0x80)  crc ^= 0x8c;
	return crc;
}

uint8_t owCrc8(uint8_t *addr, uint8_t len) {
	uint8_t crc = 0;
	while (len--) {
		crc = crcDS(*addr++, crc);
	}
	return crc;
}

void oneWireAcquireBus (OneWireDriver* drv)
{
  chMtxLock (&(drv->acquireLock));
}

void oneWireRealeaseBus (void)
{
  chMtxUnlock ();
}

/*
#                 _ __           _                    _                   
#                | '_ \         (_)                  | |                  
#                | |_) |  _ __   _   __   __   __ _  | |_     ___         
#                | .__/  | '__| | |  \ \ / /  / _` | | __|   / _ \        
#                | |     | |    | |   \ V /  | (_| | \ |_   |  __/        
#                |_|     |_|    |_|    \_/    \__,_|  \__|   \___|        
*/


static void oneWirePack (uint8_t cmd, uint8_t buffer[8])
{
  for (uint32_t i=0;  i<8; i++) {
    buffer[i] = (cmd & (1 << i)) ? 0xff : 0x00;
  }
}

static uint8_t oneWireUnpack (uint8_t buffer[8])
{
  uint8_t res = 0;

  for (uint32_t i=0;  i<8; i++) {
    if (buffer[i] == 0xff) {
      res |=  (1 << i);
    }
  }

  return res;
}

static void oneWireSerialMode (OneWireDriver* drv, SerialMode sm)
{
/* for STM32F4DISCOVERY
 *   const uint32_t modeTx =
    PAL_MODE_ALTERNATE(drv->config.dqAlternate) | PAL_STM32_OTYPE_OPENDRAIN 
    | PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_MODE_ALTERNATE;
*/
  // for STM32F103
  const uint32_t modeTx = PAL_MODE_STM32_ALTERNATE_OPENDRAIN;

  if (sm == drv->currentSm) 
    return;

  switch (sm) {
  case ModeGpioIn:
    uartStop (drv->config.uartd);
    palSetPadMode (drv->config.dqPort, drv->config.dqPad, PAL_MODE_INPUT_PULLUP);
    break;
    
  case ModeSerialReset:
//    uartStop (drv->config.uartd);
    uartStart(drv->config.uartd, &uartCfgReset);
    
    palSetPadMode (drv->config.dqPort, drv->config.dqPad, modeTx);
    break;
    
  case ModeSerialTransfert:
//    uartStop (drv->config.uartd);
    uartStart(drv->config.uartd, &uartCfgTransfert);

    palSetPadMode (drv->config.dqPort, drv->config.dqPad, modeTx);
    break;
  }
  drv->currentSm = sm;
}



static void txBufferEmpty(UARTDriver *uartp) 
{
  (void) uartp;
  OneWireDriver *drv = usartDictionnaryGetDriverByUart (uartp);
  if (drv != NULL) {
    chSysLockFromIsr();
    chBSemSignalI(&(drv->semSent));
    chSysUnlockFromIsr();
  }
}

static void rxErr(UARTDriver *uartp, uartflags_t e)
{
  (void) uartp;
  (void) e;
}


static void rxEnd(UARTDriver *uartp)
{  
  (void) uartp;
  OneWireDriver *drv = usartDictionnaryGetDriverByUart (uartp);
  if (drv != NULL) {  
    chSysLockFromIsr();
    chBSemSignalI(&(drv->semReceive));
    chSysUnlockFromIsr();
  }
}

/* static void rxChar(UARTDriver *uartp, uint16_t c) */
/* { */
/*   (void) uartp; */
/*   (void) c; */
/* } */


static uint8_t  oneWireFirst (OneWireDriver* drv, uint8_t *romBytes, 
			     bool_t conditional)
{
  drv->lastDiscrep = 0;
  drv->doneFlag = FALSE;
  // Call Next and return it's return value;
  return oneWireNext(drv, romBytes, conditional);    
}



static uint8_t oneWireNext (OneWireDriver* drv,  uint8_t *romBytes, 
			     bool_t conditional)
{
   uint8_t m = 1;             // ROM Bit index
   uint8_t n = 0;             // ROM Byte index
   uint8_t k = 1;             // Bit mask
   uint8_t x = 0;
   uint8_t discrepMarker = 0;
   uint8_t g=0;                 // Output bit
   uint8_t nxt=0;               // Return value
   short flag=0;
   static uint8_t dowcrc;

   nxt = FALSE;            // Reset next flag to false
   dowcrc = 0;             // Reset the dowcrc
   flag = !oneWireReset(drv);

   if (flag||drv->doneFlag)     // If no parts return false
   {
      drv->lastDiscrep = 0;     // Reset the search
      return FALSE;
   }
   
   // Send SearchROM or Conditional Search Rom command
   oneWireSendByte(drv, conditional ? 0xEC : 0xF0);       
   do
   {
      x = 0;
      if (oneWireReceiveBit(drv) == 1)
         x = 2;
      if (oneWireReceiveBit(drv) == 1)
         x |= 1;                    // And it's complement 
      if (x == 3)                   // There are no devices on the one wire bus
         break;
      else
      {
         if (x > 0)                 // All devices coupled have 0 or 1
            g = x >> 1;             // Bit write value for search

         // If this discrepancy is before the last discrepancy on a previous
         // Next then pick the same as last time.
         else
         {
            if (m < drv->lastDiscrep)
               g = ((romBytes[n] & k) > 0);
            // If equal to last pick 1
            else
               g = (m == drv->lastDiscrep);  // If not then pick 0

               // If 0 was picked then record position with mask k
               if (g == 0) discrepMarker = m;
         }

         // Isolate bit in ROM[n] with mask k
         if (g == 1) romBytes[n] |= k;
         else romBytes[n] &= ~k;

         oneWireSendBit(drv, g);  // ROM search write

         m++;           // Increment bit counter m
         k = k << 1;    // and shift the bit mask k
         // If the mask is 0 then go to new ROM
         if (k == 0)
         {  // Byte n and reset mask
//	    dowcrc = dscrc_table[dowcrc ^ romBytes[n]]; // Accumulate the crc
	    dowcrc = crcDS(romBytes[n], dowcrc);
            n++;
            k++;
         }
      }
   } while (n < 8);  // Loop through until through all ROM bytes 0-7

   if (m < (65||dowcrc))   // If search was unsuccessful then
      drv->lastDiscrep = 0;     // reset the last Discrepancy to zero
   else  // Search was successful, so set lastDiscrep, lastOne, nxt
   {
      drv->lastDiscrep = discrepMarker;
      drv->doneFlag = (drv->lastDiscrep == 0);
      nxt = TRUE; // Indicates search not yet complete, more parts remain
   }

   return nxt;
}   

static void usartDictionnaryAddPair (OneWireDriver* drv, UARTDriver *uart)
{
  uint8_t i = 0;

  while (i++ < ARRAY_LEN (usartDict.drv)) {
    if (usartDict.drv[i] == drv) {
      usartDict.uart[i] = uart;
      return;
    }
  }

  i = 0;
  while (i < ARRAY_LEN (usartDict.drv) && (usartDict.drv[i] != NULL)) {
    i++;
  }

  if (i < ARRAY_LEN (usartDict.drv)) {
    usartDict.drv[i] = drv;
    usartDict.uart[i] = uart;
  }
}

static OneWireDriver* usartDictionnaryGetDriverByUart (UARTDriver *uart)
{ 
  uint8_t i = 0;
  while ((i < ARRAY_LEN (usartDict.drv) && usartDict.uart[i] != uart)) {
    i++;
  }
  if (i < ARRAY_LEN (usartDict.drv)) {
    return usartDict.drv[i];
  } else {
    return NULL;
  }
}
