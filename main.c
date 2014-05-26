/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "evtimer.h"
#include "chrtclib.h"

#include "main.h"
#include "owtemp.h"
#include "oneWire.h"
#include "sensors.h"
#include "nrf24l01.h"
#include "am2302.h"
#include "bh1750.h"

#include "shell.h"

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
//#define USE_SHELL		// ChibiOS shell use

static VirtualTimer delayTimer;
static EVENTSOURCE_DECL(delay_evsrc);
static EVENTSOURCE_DECL(main_evsrc);
static EVENTSOURCE_DECL(wakeup_evsrc);
static BinarySemaphore mainsem, blinksem;

/*===========================================================================*/
/* 1-WIRE related.                                                            */
/*===========================================================================*/
OneWireDriver owDrv;
BinarySemaphore owsem;
const OneWireConfig owCfg = { .dqPort = GPIOA,
				     	 	   .dqPad =  GPIOA_USART1_TX,
				     	 	   .dqAlternate = 7,
				     	 	   .uartd = &UARTD1
							};

static uint8_t workcnt;		// work mode delay counter

// convert 6 byte array NRF24L01 address to string: AFAFAFAFAF\0
void addr_hexstr(uint8_t *addr, uint8_t *str){
    uint8_t *pin = addr;
    static const uint8_t *hex = "0123456789ABCDEF";
    uint8_t *pout = str;
    for(uint8_t i=0; i < 5; i++){
        *pout++ = hex[(*pin>>4)&0xF];
        *pout++ = hex[(*pin++)&0xF];
    }
    *pout = 0;
}

// delayTimer callback
void delayTimer_handler(void *arg){
	(void)arg;
	chSysLockFromIsr();
	chEvtBroadcastFlagsI(&delay_evsrc, (flagsmask_t)0);
	chVTSetI(&delayTimer, MS2ST(DELAYPERIOD), delayTimer_handler, 0);
	chSysUnlockFromIsr();
}

/*
 * Triggered when the NRF24L01 triggers an interrupt
 */
static void extcbnrf(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

	/*
	 * Call interrupt handler
	 */
  	chSysLockFromIsr();
	NRFReportIRQ();
	chSysUnlockFromIsr();
}

/*===========================================================================*/
/* Wakeup related.                                                     */
/*===========================================================================*/
static RTCTime timespec;
static RTCAlarm alarmspec;

/*
 * RTC callback function
 */
static void rtc_cb(RTCDriver *rtcp, rtcevent_t event) {

  (void)rtcp;

  switch (event) {
  case RTC_EVENT_OVERFLOW:
    break;
  case RTC_EVENT_SECOND:
    break;
  case RTC_EVENT_ALARM:
    break;
  }
}

/* Wake up callback.*/
static void extcbwakeup(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromIsr();
  stm32_clock_init();
  chEvtBroadcastFlagsI(&wakeup_evsrc, (flagsmask_t)0);
  extChannelDisableI(&EXTD1, 17);
  chSysUnlockFromIsr();
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, extcbnrf},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, extcbwakeup},
  }
};

#ifdef USE_SHELL
/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/
static Thread *shelltp = NULL;
#define SHELL_WA_SIZE   THD_WA_SIZE(2048)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
            states[tp->p_state]);
//                   , (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]){
  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: reboot\r\n");
    return;
  }
  chprintf(chp, "rebooting...\r\n");
  chThdSleepMilliseconds(100);
  NVIC_SystemReset();
}

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"reboot", cmd_reboot},
//  {"sleep", cmd_sleep},
//  {"date", cmd_date},
  {NULL, NULL}
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&SD2,
  commands
};
#endif	//USE_SHELL

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waBlinkerThread, 128);
__attribute__((noreturn))
static msg_t blinkerThread(void *arg) {

  (void)arg;
  chRegSetThreadName("blinkerThd");
  while (TRUE) {
	chBSemWait(&blinksem);
    palTogglePad(GPIOB, GPIOB_PIN15);
    chBSemSignal(&blinksem);
    chThdSleepMilliseconds(100);
  }
}

/*
 * Going into sleep mode.
 */
void goto_sleep(void){

  chBSemWait(&blinksem);
  palClearPad(GPIOB, GPIOB_PIN15);
  palClearPad(GPIOB, GPIOB_PIN13);

  /*
   * set NRF24L01 to power down
   */
  NRFPWRDown();

  chThdSleepMilliseconds(100);

  rtcGetTime(&RTCD1, &timespec);
  alarmspec.tv_sec = timespec.tv_sec + SLEEPTIME;
  rtcSetAlarm(&RTCD1, 0, &alarmspec);
  rtcSetCallback(&RTCD1, rtc_cb);

  extChannelEnable(&EXTD1, 17);

/* STOP mode */
  PWR->CR |= (PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
  PWR->CR &= ~PWR_CR_PDDS;

/* STANDBY mode
  PWR->CR |= (PWR_CR_PDDS | PWR_CR_CSBF | PWR_CR_CWUF);
*/

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __WFI();
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}

static BinarySemaphore evtsem;

/*
 * event manager thread
*/
WORKING_AREA(waEventThread, 1024);
__attribute__((noreturn))
static msg_t EventThread(void *arg) {
	(void)arg;
	enum{
	  	DELAY_ESID,		// id delay_evsrc
	  	WAKEUP_ESID,	// id wakeup_evsrc
	  	MAIN_ESID		// id main_evsrc
  	  	};
	static struct EventListener delay_el, wakeup_el, main_el;
	eventmask_t active;

	chRegSetThreadName("eventThd");

	chEvtInit(&delay_evsrc);
	chEvtRegister(&delay_evsrc, &delay_el, DELAY_ESID);
	chEvtInit(&wakeup_evsrc);
	chEvtRegister(&wakeup_evsrc, &wakeup_el, WAKEUP_ESID);
	chEvtInit(&main_evsrc);
	chEvtRegister(&main_evsrc, &main_el, MAIN_ESID);

	while (TRUE) {
	    chBSemWait(&evtsem);
		active = chEvtWaitAny(
								EVENT_MASK(DELAY_ESID) |
								EVENT_MASK(WAKEUP_ESID) |
								EVENT_MASK(MAIN_ESID)
								);
        // delay event listener
        if (workcnt && (active & EVENT_MASK(DELAY_ESID)) != 0){
//    		chprintf((BaseSequentialStream *)&SD2,"delay event fired\r\n");
    		//
    		uint8_t sleep_switch = palReadPad(GPIOB, GPIOB_PIN14);
//    		chprintf((BaseSequentialStream *)&SD2,"sleep switch: %d\r\n", sleep_switch);
    		if (sleep_switch && --workcnt == 0){
//        		chprintf((BaseSequentialStream *)&SD2,"going time sleep mode\r\n");
    			goto_sleep();
    		}
        }

        // wakeup event listener
        if ((active & EVENT_MASK(WAKEUP_ESID)) != 0){
//    		chprintf((BaseSequentialStream *)&SD2,"wakeup event fired\r\n");
    		palSetPad(GPIOB, GPIOB_PIN13);

    		chThdSleepSeconds(1);

    		chBSemSignal(&blinksem);
    		NRFPWRUp();
    		workcnt = WORKTIME; 	// set working time
    		chBSemSignal(&mainsem);
        }

        // main event listener
        if ((active & EVENT_MASK(MAIN_ESID)) != 0){
//    		chprintf((BaseSequentialStream *)&SD2,"main event fired\r\n");
    		uint8_t sleep_switch = palReadPad(GPIOB, GPIOB_PIN14);
//    		chprintf((BaseSequentialStream *)&SD2,"sleep switch: %d\r\n", sleep_switch);
    		if (sleep_switch){
//        		chprintf((BaseSequentialStream *)&SD2,"going main sleep mode\r\n");
    			goto_sleep();
    		}
    		else{
    			chBSemSignal(&mainsem);
    		}
        }

        chSysLock();
    	chEvtGetAndClearFlagsI(&delay_el);
    	chEvtGetAndClearFlagsI(&wakeup_el);
    	chEvtGetAndClearFlagsI(&main_el);
    	chSysUnlock();

        chBSemSignal(&evtsem);
	} //while
}

// called on kernel panic
void port_halt(void){
	port_disable();
	palSetPad(GPIOB, GPIOB_PIN15); // turn on error
	while(TRUE)
	{
	}
}

#define SENSOR_READ_MAX		5
#define NRF_SEND_MAX		5

bool_t nrf_send_msg(MESSAGE_T *msg){

	uint8_t sendcnt = NRF_SEND_MAX;
	while (sendcnt-- > 0){
		if (NRFSendData((uint8_t *) msg)){
			return TRUE;
		}
	}
	return FALSE;
}

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, GPIOA_PIN2, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(GPIOA, GPIOA_PIN3, PAL_MODE_INPUT);

  chprintf((BaseSequentialStream *)&SD2,"Remote sensor module, F/W:%s\r\n", FIRMWARE);

#ifdef USE_SHELL
  /*
   * Shell manager initialization.
   */
  shellInit();
#endif

  workcnt = WORKTIME; 	// set working time

  // sleep mode switch
  palSetPadMode(GPIOB, GPIOB_PIN14, PAL_MODE_INPUT);
  chBSemInit(&mainsem, TRUE);

  /*
   * Setup NRF24L01 IRQ pad.
   */
  palSetPadMode(NRF_PORT_CE_IRQ, NRF_PORT_IRQ, PAL_MODE_INPUT);
  /*
   * Activates the EXT driver 1.
   */
  extStart(&EXTD1, &extcfg);
  /*
   * Enable NRF24L01 interrupts.
   */
  extChannelEnable(&EXTD1, NRF_PORT_IRQ);

  // init event semaphore
  chBSemInit(&evtsem, TRUE);

  /*
   * NRF24L01+ device initialization
   */
  NRFInit();

  static uint8_t txaddr[5], rxaddr[5], rxaddrstr[11] = {'\0'}, txaddrstr[11] = {'\0'};
  NRFGetAddrs(txaddr, rxaddr);
  addr_hexstr(rxaddr, rxaddrstr);
  addr_hexstr(txaddr, txaddrstr);
  chprintf((BaseSequentialStream *)&SD2,"CHANNEL: %d, TX ADDR:%s, RX ADDR:%s\r\n", CHANNEL, txaddrstr, rxaddrstr);

  // sensor power pin PAL mode
  palSetPadMode(GPIOB, GPIOB_PIN13, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOB, GPIOB_PIN13);

  // LED pin PAL mode
  palSetPadMode(GPIOB, GPIOB_PIN15, PAL_MODE_OUTPUT_PUSHPULL);
  /*
   * Creates the blinker thread.
   */
  chBSemInit(&blinksem, FALSE);
  chThdCreateStatic(waBlinkerThread, sizeof(waBlinkerThread), NORMALPRIO, blinkerThread, NULL);

  // Creates event manager thread.
  chThdCreateStatic(waEventThread, sizeof(waEventThread), NORMALPRIO+1, EventThread, NULL);

  // start event manager thread
  chBSemSignal(&evtsem);

  // init 1-wire sensors
  ow_temp_init();
  chBSemInit(&owsem,FALSE);

  // init ADC1
//  sensors_init();

  // init DHT/AM2301 temperature/humidity sensors
  dht_init();

  // init BH1750 light sensor
  bh1750_init();

  // start 60s delay timer
  chVTSet(&delayTimer, MS2ST(DELAYPERIOD), delayTimer_handler, 0);

  // start main thread
  chBSemSignal(&mainsem);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */
  while (TRUE) {
#ifdef USE_SHELL
    if (!shelltp)
    	//&& (SDU1.config->usbp->state == USB_ACTIVE))
      shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
    else if (chThdTerminated(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
#endif
	MESSAGE_T msg = {0};
    uint8_t readcnt;
    bool_t read_ok;

    chBSemWait(&mainsem);

    // BH1750
    int16_t light;
    readcnt = SENSOR_READ_MAX;
    read_ok = FALSE;
    while (readcnt-- > 0){
    	read_ok = bh1750_read(&light) == BH1750_NO_ERROR;
    	if (read_ok) break;
    }
	msg.msgType = 2;
	msg.sensorType = 1;
    if (read_ok){
		msg.msgType = 1;
		msg.data.iValue = light;
		chprintf((BaseSequentialStream *)&SD2,"BH1750, light: %d\r\n", light);
    }
    else{
		chprintf((BaseSequentialStream *)&SD2,"BH1750, read error\r\n");
    }
	if (nrf_send_msg(&msg)){
    		chprintf((BaseSequentialStream *)&SD2,"BH1750, send ok\r\n");
    }
	else{
		chprintf((BaseSequentialStream *)&SD2,"BH1750, send fail\r\n");
	}
	// DHT_1
    int temperature, humidity;
    readcnt = SENSOR_READ_MAX;
    read_ok = FALSE;
    while (readcnt-- > 0){
    	read_ok = dht_read(1, &temperature, &humidity) == DHT_NO_ERROR;
    	if (read_ok) break;
    }
	msg.msgType = 2;
	msg.sensorType = 2;
	msg.owkey[0] = 1;
    if (!read_ok){
        chprintf((BaseSequentialStream *)&SD2,"DHT1, read fail\r\n");
    	if (nrf_send_msg(&msg)){
            chprintf((BaseSequentialStream *)&SD2,"DHT1, send ok\r\n");
    	}
    	else{
            chprintf((BaseSequentialStream *)&SD2,"DHT1, send fail\r\n");
    	}
    }
    else{
        chprintf((BaseSequentialStream *)&SD2,"DHT1, temperature: %.2f, humidity: %.2f\r\n", (float)temperature/10, (float)humidity/10);
    	msg.msgType = 1;
		msg.valueType = 0;
		msg.data.iValue = temperature;
		if (nrf_send_msg(&msg)){
    		chprintf((BaseSequentialStream *)&SD2,"DHT1, temperature send ok\r\n");
		}
		else{
    		chprintf((BaseSequentialStream *)&SD2,"DHT1, temperature send fail\r\n");
		}
		msg.valueType = 1;
		msg.data.iValue = humidity;
		if (nrf_send_msg(&msg)){
    		chprintf((BaseSequentialStream *)&SD2,"DHT1, humidity send ok\r\n");
		}
		else{
    		chprintf((BaseSequentialStream *)&SD2,"DHT1, humidity send fail\r\n");
		}
    }
    // DHT_2
    readcnt = SENSOR_READ_MAX;
    read_ok = FALSE;
    while (readcnt-- > 0){
    	read_ok = dht_read(2, &temperature, &humidity) == DHT_NO_ERROR;
    	if (read_ok) break;
    }
	msg.msgType = 2;
	msg.sensorType = 2;
	msg.owkey[0] = 2;
    if (!read_ok){
        chprintf((BaseSequentialStream *)&SD2,"DHT2, read fail\r\n");
    	if (nrf_send_msg(&msg)){
            chprintf((BaseSequentialStream *)&SD2,"DHT2, send ok\r\n");
    	}
    	else{
            chprintf((BaseSequentialStream *)&SD2,"DHT2, send fail\r\n");
    	}
    }
    else{
        chprintf((BaseSequentialStream *)&SD2,"DHT2, temperature: %.2f, humidity: %.2f\r\n", (float)temperature/10, (float)humidity/10);
    	msg.msgType = 1;
		msg.valueType = 0;
		msg.data.iValue = temperature;
		if (nrf_send_msg(&msg)){
    		chprintf((BaseSequentialStream *)&SD2,"DHT2, temperature send ok\r\n");
		}
		else {
    		chprintf((BaseSequentialStream *)&SD2,"DHT2, temperature send fail\r\n");
		}
		msg.valueType = 1;
		msg.data.iValue = humidity;
		if (nrf_send_msg(&msg)){
    		chprintf((BaseSequentialStream *)&SD2,"DHT2, humidity send ok\r\n");
		}
		else{
    		chprintf((BaseSequentialStream *)&SD2,"DHT2, humidity send fail\r\n");
		}
    }

    // DS1820B
    readcnt = SENSOR_READ_MAX;
    read_ok = FALSE;
    while (readcnt-- > 0){
    	read_ok = owtemp_read() == OW_TEMP_NO_ERROR;
    	if (read_ok) break;
    }
	msg.msgType = 2;
	msg.sensorType = 0;
    if (!read_ok){
		chprintf((BaseSequentialStream *)&SD2,"DS1820, read fail\r\n");
    	if (nrf_send_msg(&msg)){
    		chprintf((BaseSequentialStream *)&SD2,"DS1820, send ok\r\n");
    	}
    	else{
    		chprintf((BaseSequentialStream *)&SD2,"DS1820, send fail\r\n");
    	}
    }
    else{
	    for (uint8_t i=0; i < ARRAY_LEN(ow_temp_read.owtemp); i++){
	    	if (ow_temp_read.owtemp[i].key[0] == 0x28){	// DS1820B
	    		chprintf((BaseSequentialStream *)&SD2,"DS1820, temperature: %.2f\r\n", ow_temp_read.owtemp[i].value);
	    		msg.msgType = 1;
	    		memcpy(&(msg.owkey),&(ow_temp_read.owtemp[i].key),sizeof(msg.owkey));
	    		msg.data.fValue = ow_temp_read.owtemp[i].value;
	    		if (nrf_send_msg(&msg)){
		    		chprintf((BaseSequentialStream *)&SD2,"DS1820, send ok\r\n");
	    		}
	    		else{
		    		chprintf((BaseSequentialStream *)&SD2,"DS1820, send fail\r\n");
	    		}
	    	}
	    }
    }
    // BATTERY
/*	readcnt = SENSOR_READ_MAX;
    read_ok = FALSE;
    while (readcnt-- > 0){
    	read_ok = sensors_read() == ADC_NO_ERROR;
    	if (read_ok) break;
    }
	msg.msgType = 2;
	msg.sensorType = 3;
    if (!read_ok){
    	nrf_send_msg(&msg);
    }
    else{
  		chprintf((BaseSequentialStream *)&SD2,"BATTERY, value: %d\r\n", sensors.sensor[BATTSENSOR].value);
		if (sensors.sensor[BATTSENSOR].value < BATTERYLOW) {
	    	msg.msgType = 0;
	    	msg.errorCode = 1;
	    	msg.data.iValue = sensors.sensor[BATTSENSOR].value;
		}
		else{
	    	msg.msgType = 0;
	    	msg.errorCode = 0;
	    	msg.data.iValue = sensors.sensor[BATTSENSOR].value;
		}
		if (nrf_send_msg(&msg)){
    		chprintf((BaseSequentialStream *)&SD2,"BATTERY, send ok\r\n");
		}
*/
/*    	// other ADC
	    for (uint8_t i=0; i < SENSORSNUM; i++){
	  		chprintf((BaseSequentialStream *)&SD2,"ADC, sensor[%d]: %d\r\n", i, sensors.sensor[i].value);
	    	msg.msgType = 1;
    		msg.sensorType = 1;
    		msg.valueType = 2;
    		msg.data.iValue = sensors.sensor[i].value;
    		if (NRFSendData((uint8_t *) &msg)){
        		chprintf((BaseSequentialStream *)&SD2,"SENSOR[%d], send ok\r\n", i);
    		}
	    }
    }*/

    chEvtBroadcastFlags(&main_evsrc, (flagsmask_t)0);
    chThdSleepMilliseconds(100);
  }
}
