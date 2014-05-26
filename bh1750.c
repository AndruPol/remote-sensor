/*
 * bh1750.c
 */

#include "ch.h"
#include "hal.h"

#include "bh1750.h"

#define I2CD_BH1750			I2CD1

#define BH1750_ADDR 		0x23 // device address
#define BH1750_PWR_DOWN		0x0	 // No active state.
#define BH1750_CONT_HMODE	0x10 // Continuously H-Resolution Mode
#define BH1750_DELAY		200	 // wait results (datasheet says max. 180ms)
#define BH1750_TIMEOUT_MS	10	 // i2c transmit timeout

static const I2CConfig i2cfg = { OPMODE_I2C, 100000, STD_DUTY_CYCLE, };

// read data
bh1750_error_t bh1750_read(int16_t *value) {
  static uint8_t txbuf[2] = {0}, rxbuf[2] = {0};

  bh1750_error_t err = BH1750_NO_ERROR;

  i2cAcquireBus(&I2CD_BH1750);
  txbuf[0] = BH1750_CONT_HMODE;
  if (i2cMasterTransmitTimeout(&I2CD_BH1750, BH1750_ADDR, txbuf, 1, rxbuf, 0, MS2ST(BH1750_TIMEOUT_MS)) != RDY_OK){
	  err = BH1750_TIMEOUT;
	  goto error;
  }
  chThdSleepMilliseconds(BH1750_DELAY);
  txbuf[0] = BH1750_PWR_DOWN;
  if (i2cMasterTransmitTimeout(&I2CD_BH1750, BH1750_ADDR, txbuf, 1, rxbuf, 2, MS2ST(BH1750_TIMEOUT_MS)) != RDY_OK){
	  err = BH1750_TIMEOUT;
  }

error:
  i2cReleaseBus(&I2CD_BH1750);

  if (err != BH1750_NO_ERROR){
	  return err;
  }

  *value = ((rxbuf[0] << 8) + rxbuf[1])/1.2;
  return BH1750_NO_ERROR;
}

void bh1750_init(void) {
  // Start the i2c driver
  i2cStart(&I2CD_BH1750, &i2cfg);
  palSetPadMode(GPIOB, GPIOB_PIN6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
  palSetPadMode(GPIOB, GPIOB_PIN7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
}
