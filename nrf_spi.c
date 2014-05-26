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

#include "ch.h"
#include "hal.h"

#include "nrf_spi.h"

#include <string.h>

/*
 * SPI speed definitions.
 */
#define   SPI_DIV2                  0
#define   SPI_DIV4                  (SPI_CR1_BR_0)
#define   SPI_DIV8                  (SPI_CR1_BR_1)
#define   SPI_DIV16                 (SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define   SPI_DIV32                 (SPI_CR1_BR_2)
#define   SPI_DIV64                 (SPI_CR1_BR_2 | SPI_CR1_BR_0)

#define   HIGHSPEED_SPI_DIV      	SPI_DIV2

/*
 * Mutex to lock output buffer
 */
static Mutex					SPIMtx; /* Mutex */

/*
 * SPI configuration structure.
 */
static const SPIConfig hs_spicfg = {NULL, GPIOA, GPIOA_PIN4, SPI_DIV2};

/*
 * Initialize the SPI interface
 */
void SPIInit(void)
{
	/*
	 * SPI1 I/O pins setup.
	 */
	palSetPadMode(GPIOA, GPIOA_PIN5, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* SCK. */
	palSetPadMode(GPIOA, GPIOA_PIN6, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MISO.*/
	palSetPadMode(GPIOA, GPIOA_PIN7, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MOSI.*/
	palSetPadMode(GPIOA, GPIOA_PIN4, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(GPIOA, GPIOA_PIN4);

	/*
	 * Initialize Mutex
	 */
	chMtxInit(&SPIMtx); /* Mutex initialization before use */
}

/*
 * SPI bus exchange routine
 */
int SPIExchangeData(SPIDriver *spip, uint8_t *tx, uint8_t *rx, size_t size) {
	chMtxLock(&SPIMtx);

	/*
	 * Do exchange between device and MCU.
	 */
	spiAcquireBus(spip);              /* Acquire ownership of the bus.    */
	spiStart(spip, &hs_spicfg);       /* Setup transfer parameters.       */
	spiSelect(spip);                  /* Slave Select assertion.          */
	spiExchange(spip, size, tx, rx);  /* Atomic transfer operations.      */
	spiUnselect(spip);                /* Slave Select de-assertion.       */
	spiReleaseBus(spip);              /* Ownership release.               */

	chMtxUnlock();

	return 0;
}

/*
 * SPI bus send routine
 */
int SPISendData(SPIDriver *spip, uint8_t *tx, size_t size) {

	chMtxLock(&SPIMtx);

	/*
	 * Do exchange between device and MCU.
	 */
	spiAcquireBus(spip);              /* Acquire ownership of the bus.    */
	spiStart(spip, &hs_spicfg);       /* Setup transfer parameters.       */
	spiSelect(spip);                  /* Slave Select assertion.          */
	spiSend(spip, size, tx);					/* Send command											*/
	spiUnselect(spip);                /* Slave Select de-assertion.       */
	spiReleaseBus(spip);              /* Ownership release.               */

	chMtxUnlock();

	return 0;
}

/*
 * SPI bus receive routine
 */
int SPIReceiveData(SPIDriver *spip, uint8_t *rx, size_t size) {

	chMtxLock(&SPIMtx);

	/*
	 * Do exchange between device and MCU.
	 */
	spiAcquireBus(spip);              /* Acquire ownership of the bus.    */
	spiStart(spip, &hs_spicfg);       /* Setup transfer parameters.       */
	spiSelect(spip);                  /* Slave Select assertion.          */
	spiReceive(spip, size, rx);
	spiUnselect(spip);                /* Slave Select de-assertion.       */
	spiReleaseBus(spip);              /* Ownership release.               */

	chMtxUnlock();

	return 0;
}
