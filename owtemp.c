/*
 * owtemp.c - 1-Wire temperature sensor DS1820B driver
 *
 *  Created on: 28.03.2013
 *      Author: A.Polyakov
*/

#include "ch.h"
#include "hal.h"

#include "owtemp.h"
#include "oneWire.h"
#include "ds1820b.h"

#include <stdlib.h>
#include <string.h>

//#define ARRAY_LEN(a) 		(sizeof(a)/sizeof(a[0]))
#define OWSCAN_TIMEOUT_MS	250	// задержка сканирования датчиков температуры (мсек)
#define OWTEMPCONV			190	// задержка конвертирования (мсек) для датчиков температуры DS1820B
#define OWTEMPPREC			10	// точность преобразования для датчиков температуры

static OneWireRomAddress romAddr[OWDEVICES];

// признак инициализации oneWire
extern bool_t ow_initialized;

extern OneWireDriver owDrv;
extern const OneWireConfig owCfg;
extern BinarySemaphore owsem;

ow_temp_read_t	ow_temp_read;	// возвращает значения температуры и признак ошибки
static BinarySemaphore owtempsem;

/*
 * Процесс сканирования температурных датчиков DS1820B
*/
static Thread *OWTempThread_p;
static WORKING_AREA(waOWTempThread, 256);
__attribute__((noreturn))
static msg_t OWTempThread(void *arg) {
  (void)arg;
  chRegSetThreadName("owTempThd");
  while (TRUE) {
	ow_temp_read_t *req;
	Thread *tp;

	tp = chMsgWait();
	req = (ow_temp_read_t *) chMsgGet(tp);
	chMsgRelease(tp, (msg_t) req);

	req->error = OW_TEMP_NO_ERROR;

	chBSemWait(&owsem);
	oneWireSearchRom (&owDrv, FALSE, romAddr, OWDEVICES);
	bool_t ow_found = FALSE;
    for (uint8_t i=0; i<OWDEVICES; i++) {
      if (romAddr[i].addr[0] != 0x28) continue;
      ow_found = TRUE;
      ds1820BInit (&owDrv, &(romAddr[i]), OWTEMPPREC);
      ds1820BAskTemp (&owDrv, &(romAddr[i]));
    }
	chBSemSignal(&owsem);

    if (ow_found){
    	chThdSleepMilliseconds(OWTEMPCONV);
    	chBSemWait(&owsem);
        for (uint8_t i=0; i<OWDEVICES; i++) {
          req->owtemp[i].key[0] = 0x00;
          if (romAddr[i].addr[0] == 0x28) {
        	memcpy(&req->owtemp[i].key,&romAddr[i],8);
        	req->owtemp[i].value = ds1820BGGetTempFromRam(&owDrv, &(romAddr[i]));
          }
        }
    	chBSemSignal(&owsem);
    }
    else {
    	req->error = OW_TEMP_NOT_FOUND;
    }

	chBSemSignal(&owtempsem);
  }//while
}

// получает значения с температурных датчиков DS1820B
ow_temp_error_t owtemp_read(void) {
	ow_temp_read_t *ow_temp_read_p = &ow_temp_read;

	chBSemWait(&owtempsem); /* to be sure */

	chMsgSend(OWTempThread_p, (msg_t) ow_temp_read_p);

	/* wait for reply */
	if(chBSemWaitTimeout(&owtempsem, MS2ST(OWSCAN_TIMEOUT_MS)) == RDY_TIMEOUT) {
		chBSemReset(&owtempsem, FALSE);
		ow_temp_read.error = OW_TEMP_TIMEOUT;
		return OW_TEMP_TIMEOUT;
	}
	chBSemReset(&owtempsem, FALSE);
	return ow_temp_read.error;
}

// создает процесс опроса датчиков DS1820B
void ow_temp_init(void){
	chBSemInit(&owtempsem, FALSE);
	oneWireInit(&owDrv,&owCfg);
	// Создаем процесс опроса датчиков 1-wire
	OWTempThread_p = chThdCreateStatic(waOWTempThread, sizeof(waOWTempThread), OWTEMP_PRIO, OWTempThread, NULL);
}
