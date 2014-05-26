/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#define CHANNEL				0

/*
 * The number of bytes the NRF24L01 RX FIFO is going to hold
 * This can be 32 bytes MAX
 */
#define NRF_FIFO_BYTES 		16

/*
 * Define IRQ an CE PORT/PAD
 * Make sure to setup the IRQ pad with the EXT driver as well
 */
#define NRF_PORT_CE_IRQ		GPIOB
#define NRF_PORT_IRQ		9
#define NRF_PORT_CE			8

/**
 * @name    NRF status flags
 * @{
 */
#define NRF_TX_NO_ERROR			0  /**< @brief No pending conditions.      */
#define NRF_TX_OVERRUN_ERROR	2  /**< @brief Overflow happened.          */
#define NRF_TRANSMIT_ERROR		4  /**< @brief Transmission error.         */
/** @} */

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  NRF_UNINIT = 0,                  /**< Not initialized.                   */
  NRF_STOP = 1,                    /**< Stopped.                           */
  NRF_READY = 2                    /**< Ready.                             */
} nrfstate_t;

/**
 * @brief   Transmitter state machine states.
 */
typedef enum {
  NRF_TX_IDLE = 0,                 /**< Not transmitting.                  */
  NRF_TX_ACTIVE = 1,               /**< Transmitting.                      */
  NRF_TX_COMPLETE = 2              /**< Buffer complete.                   */
} nrftxstate_t;

/**
 * @brief   Receiver state machine states.
 */
typedef enum {
  NRF_RX_IDLE = 0,                 /**< Not receiving.                     */
  NRF_RX_ACTIVE = 1,               /**< Receiving.                         */
  NRF_RX_COMPLETE = 2              /**< Buffer complete.                   */
} nrfrxstate_t;

/**
 * @brief   NRF driver condition flags type.
 */
typedef uint8_t nrfflags_t;

typedef struct _NRFD NRFD;
struct _NRFD {
  /**
   * @brief Driver state.
   */
  volatile nrfstate_t        state;
  /**
   * @brief Transmitter state.
   */
  volatile nrftxstate_t      txstate;
  /**
   * @brief Receiver state.
   */
  volatile nrfrxstate_t      rxstate;
  /**
   * @brief   NRF driver condition flags type.
   */
  volatile nrfflags_t		flags;
  /*
   * Binary semaphore, to lock access to the send and receive queue of the NRF24L01
   */
  BinarySemaphore 	NRFSemIRQ;
  BinarySemaphore	NRFSemRX;
  BinarySemaphore	NRFSemTX;
};

extern NRFD nrf;

/*
 * Initialize the NRF24L01 Wifi chip.
 */
void NRFInit(void);

/*
 * Set channel 
 * Frequency is F0= 2400 + RF_CH [MHz]
 */
void NRFSetChannel(uint8_t chan);

/*
 * Handle the IRQ signal, unlock the Semaphores or set flags. 
 */
void NRFHandleIrq(void);

/*
 * NRF24L01 test
 */
void NRFtest(void);

/*
 * Function to send data
 * This functions blocks until data is available
 * The send output buffer needs to be NRF_FIFO_BYTES(32) bytes wide
 */
bool_t NRFSendData(uint8_t *inBuf);

/*
 * Routine to unlock IRQ handling.
 */
void NRFReportIRQ(void);

/*
 * Test if there is a carrier signal active on the channel
 * When present return true
 */
uint8_t NRFCarrier(void);

/*
 * Function to receive data from FIFO
 * This functions blocks until data is available
 * The output buffer needs to be NRF_FIFO_BYTES(32) bytes wide
 */
void NRFReceiveData(uint8_t *pipeNr, uint8_t *outBuf);

/*
 * Set the address to the receiver pipe
 * Normaly pipe  is used to receive the ack packets by shockburst
 * Use pipe 1 as the first data receive pipe
 * @Arguments
 * pipe				Pipe number to set the address to
 * addr_size	The size of the address in bytes
 * addr				Byte array holding the addr, LSB first
 */
void NRFSetRecvAddr(uint8_t pipe, uint8_t addr[], uint8_t addrSize);


/*
 * Set the address to the receiver pipe
 * The receive address of pipe 0 is set as well, to enable auto ACK handling
 * @Arguments
 * pipe				Pipe number to set the address to
 * addr_size	The size of the address in bytes
 * addr				Byte array holding the address, LSB first
 */
void NRFSetSendAddr(uint8_t addr[], uint8_t addrSize);

void NRFPWRDown(void);
void NRFPWRUp(void);
void NRFGetAddrs(uint8_t *txaddr, uint8_t *rxaddr);
uint8_t NRFChannelScan(uint8_t chan);

/*
 * NRF thread
 */
#define NRF_WA_SIZE		256
extern WORKING_AREA(NRFThreadWA, NRF_WA_SIZE);
extern msg_t NRFThread(void *arg);
