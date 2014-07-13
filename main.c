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
#include "nrf24l01.h"
#include "am2302.h"
#include "bh1750.h"

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

#define DEBUG		0	// print debug to console
#define USE_SHELL	0	// ChibiOS shell use
#define ADC			0
#define AES			1	// encrypt message
#define STANDBY		1	// standby power mode

#if USE_SHELL
#include "shell.h"
#endif

#if ADC
#include "sensors.h"
#endif

#if AES
#include "aes/inc/aes.h"
#include "aes/inc/aes_user_options.h"
#include "aes_secret.h"
static aes_data_t aes_data;
#endif

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
				     	 	   .dqAlternate = 7,	// not used for stm32f1xx
				     	 	   .uartd = &UARTD1
							};

// convert 5 byte array NRF24L01 address to string: AFAFAFAFAF\0
static void addr_hexstr(uint8_t *addr, uint8_t *str){
    uint8_t *pin = addr;
    const char *hex = "0123456789ABCDEF";
    uint8_t *pout = str;
    for(uint8_t i=0; i < 5; i++){
        *pout++ = hex[(*pin>>4)&0xF];
        *pout++ = hex[(*pin++)&0xF];
    }
    *pout = 0;
}

// convert 1-wire 8 bytes address to char[17]
static void owkey_hexstr(uint8_t *addr, uint8_t *str){
    uint8_t *pin = addr;
    const char *hex = "0123456789ABCDEF";
    uint8_t *pout = str;
    for(uint8_t i=0; i < 8; i++){
        *pout++ = hex[(*pin>>4)&0xF];
        *pout++ = hex[(*pin++)&0xF];
    }
    *pout = 0;
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
/* Wakeup related.                                                    		 */
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

#if (STANDBY == FALSE)
  chSysLockFromIsr();
  stm32_clock_init();
  chEvtBroadcastFlagsI(&wakeup_evsrc, (flagsmask_t)0);
  extChannelDisableI(&EXTD1, 17);
  chSysUnlockFromIsr();
#endif
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

#if USE_SHELL
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

  chThdSleepMilliseconds(2);

  chSysLock();
  rtcGetTime(&RTCD1, &timespec);
  alarmspec.tv_sec = timespec.tv_sec + SLEEPTIME;
  rtcSetAlarm(&RTCD1, 0, &alarmspec);
  rtcSetCallback(&RTCD1, rtc_cb);

  extChannelEnable(&EXTD1, 17);

#if STANDBY
  /* STANDBY mode */
  PWR->CR |= (PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
#else
  /* STOP mode */
  PWR->CR |= (PWR_CR_LPDS | PWR_CR_CSBF | PWR_CR_CWUF);
  PWR->CR &= ~PWR_CR_PDDS;
#endif

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __WFI();
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
	  	WAKEUP_ESID,	// id wakeup_evsrc
	  	MAIN_ESID		// id main_evsrc
  	  	};
	static struct EventListener delay_el, wakeup_el, main_el;
	eventmask_t active;

	chRegSetThreadName("eventThd");

	chEvtInit(&wakeup_evsrc);
	chEvtRegister(&wakeup_evsrc, &wakeup_el, WAKEUP_ESID);
	chEvtInit(&main_evsrc);
	chEvtRegister(&main_evsrc, &main_el, MAIN_ESID);

	while (TRUE) {
	    chBSemWait(&evtsem);
		active = chEvtWaitAny(
								EVENT_MASK(WAKEUP_ESID) |
								EVENT_MASK(MAIN_ESID)
								);

		// wakeup event listener
        if ((active & EVENT_MASK(WAKEUP_ESID)) != 0){
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"wakeup event fired\r\n");
#endif
#if (STANDBY == FALSE)
    		palSetPad(GPIOB, GPIOB_PIN13);
    		chBSemSignal(&blinksem);
    		NRFPWRUp();
    		chThdSleepMilliseconds(100);
#endif
    		chBSemSignal(&mainsem);
        }

        // main event listener
        if ((active & EVENT_MASK(MAIN_ESID)) != 0){
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"main event fired\r\n");
#endif
    		uint8_t sleep_switch = palReadPad(GPIOB, GPIOB_PIN14);
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"sleep switch: %d\r\n", sleep_switch);
#endif
    		if (sleep_switch){
#if DEBUG
        		chprintf((BaseSequentialStream *)&SD2,"going to sleep mode\r\n");
#endif
    			goto_sleep();
    		}
    		else{
    			chBSemSignal(&mainsem);
    		}
        }

        chSysLock();
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
#if AES
	uint8_t buf[MSGLEN];
	aes_encrypt_ecb(&aes_data, msg, buf);
#endif
	while (sendcnt-- > 0){
#if AES
		if (NRFSendData((uint8_t *) buf)){
#else
		if (NRFSendData((uint8_t *) msg)){
#endif
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

  chprintf((BaseSequentialStream *)&SD2,"\r\nRemote sensor module, F/W:%s\r\n", FIRMWARE);

  // sensor power pin PAL mode
  palSetPadMode(GPIOB, GPIOB_PIN13, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOB, GPIOB_PIN13);

  // sleep mode switch
  palSetPadMode(GPIOB, GPIOB_PIN14, PAL_MODE_INPUT);

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
  /*
   * NRF24L01+ device initialization
   */
  NRFInit();

  static uint8_t txaddr[5], rxaddr[5], rxaddrstr[11] = {'\0'}, txaddrstr[11] = {'\0'};
  NRFGetAddrs(txaddr, rxaddr);
  addr_hexstr(rxaddr, rxaddrstr);
  addr_hexstr(txaddr, txaddrstr);
  chprintf((BaseSequentialStream *)&SD2,"CHANNEL: %d, TX ADDR:%s, RX ADDR:%s\r\n", CHANNEL, txaddrstr, rxaddrstr);

  // LED pin PAL mode
  palSetPadMode(GPIOB, GPIOB_PIN15, PAL_MODE_OUTPUT_PUSHPULL);
  /*
   * Creates the blinker thread.
   */
  chBSemInit(&blinksem, FALSE);
  chThdCreateStatic(waBlinkerThread, sizeof(waBlinkerThread), NORMALPRIO, blinkerThread, NULL);

  // init event semaphore
  chBSemInit(&evtsem, FALSE);

  // Creates event manager thread.
  chThdCreateStatic(waEventThread, sizeof(waEventThread), NORMALPRIO+1, EventThread, NULL);

  // init 1-wire sensors
  ow_temp_init();
  chBSemInit(&owsem,FALSE);

#if ADC
  // init ADC1
//  sensors_init();
#endif

  // init DHT21/22 temperature/humidity sensors
  dht_init();

  // init BH1750 light sensor
  bh1750_init();

#if AES
  aes_initialize(&aes_data, AES_KEY_LENGTH_128_BITS, aes_key, NULL);
#endif

  // start main thread
  chBSemInit(&mainsem, FALSE);

#if USE_SHELL
  /*
   * Shell manager initialization.
   */
  shellInit();
#endif

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */
  while (TRUE) {
#if USE_SHELL
    if (!shelltp)
      shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
    else if (chThdTerminated(shelltp)) {
      chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
#endif
	MESSAGE_T msg = {0};
	msg.sensorID = SENSORID;

    uint8_t readcnt;
    bool_t read_ok;

    chBSemWait(&mainsem);

    // BH1750
    int16_t light;
    readcnt = SENSOR_READ_MAX;
    read_ok = FALSE;
    while (readcnt-- > 0) {
    	msg.data.cValue[0] = bh1750_read(&light);
    	read_ok = msg.data.cValue[0] == BH1750_NO_ERROR;
    	if (read_ok) break;
    }
	msg.msgType = SENSOR_ERROR;
	msg.sensorType = BH1750;
	msg.valueType = LIGHT;
    if (read_ok){
		msg.msgType = SENSOR_DATA;
		msg.data.iValue = light;
		chprintf((BaseSequentialStream *)&SD2,"SENSOR:%d:BH1750:1:LIGHT:%d\r\n", SENSORID, light);
    }
    else{
		chprintf((BaseSequentialStream *)&SD2,"ERROR:%d:BH1750:1:%d\r\n", SENSORID, msg.data.cValue[0]);
    }
	if (nrf_send_msg(&msg)){
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"BH1750, send ok\r\n");
#endif
    }
	else{
#if DEBUG
		chprintf((BaseSequentialStream *)&SD2,"BH1750, send fail\r\n");
#endif
	}

    // DS1820B
    readcnt = SENSOR_READ_MAX;
    read_ok = FALSE;
    while (readcnt-- > 0){
    	msg.data.cValue[0] = owtemp_read();
    	read_ok = msg.data.cValue[0] == OW_TEMP_NO_ERROR;
    	if (read_ok) break;
    }
	msg.msgType = SENSOR_ERROR;
	msg.sensorType = DS1820;
	msg.valueType = TEMPERATURE;
    if (!read_ok){
		chprintf((BaseSequentialStream *)&SD2,"ERROR:%d:DS1820:0:%d\r\n", SENSORID, msg.data.cValue[0]);
    	if (nrf_send_msg(&msg)){
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"DS1820, error send ok\r\n");
#endif
    	}
    	else{
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"DS1820, error send fail\r\n");
#endif
    	}
    }
    else{
	    for (uint8_t i=0; i < ARRAY_LEN(ow_temp_read.owtemp); i++){
	    	if (ow_temp_read.owtemp[i].key[0] == 0x28){	// DS1820B
	    		uint8_t owaddr[17] = {0};
	    		owkey_hexstr(ow_temp_read.owtemp[i].key, owaddr);
	    		msg.msgType = SENSOR_DATA;
	    		memcpy(&(msg.owkey),&(ow_temp_read.owtemp[i].key),sizeof(msg.owkey));
	    		msg.data.fValue = ow_temp_read.owtemp[i].value;
	    		chprintf((BaseSequentialStream *)&SD2,"SENSOR:%d:DS1820:%s:TEMPERATURE:%.2f\r\n", SENSORID, owaddr, ow_temp_read.owtemp[i].value);
	    		if (nrf_send_msg(&msg)){
#if DEBUG
	    			chprintf((BaseSequentialStream *)&SD2,"DS1820[%s], temperature send ok\r\n", owaddr);
#endif
	    		}
	    		else{
#if DEBUG
	    			chprintf((BaseSequentialStream *)&SD2,"DS1820[%s], temperature send fail\r\n", owaddr);
#endif
	    		}
	    	}
	    }
    }

    // DHT21 power on timeout
    chThdSleepMilliseconds(1000);

	// DHT_1
    int temperature, humidity;
    readcnt = SENSOR_READ_MAX;
    read_ok = FALSE;
    while (readcnt-- > 0){
    	msg.data.cValue[0] = dht_read(1, &temperature, &humidity);
    	read_ok = msg.data.cValue[0] == DHT_NO_ERROR;
    	if (read_ok) break;
    }
	msg.msgType = SENSOR_ERROR;
	msg.sensorType = DHT;
	msg.owkey[0] = 1;
    if (!read_ok){
        chprintf((BaseSequentialStream *)&SD2,"ERROR:%d:DHT:1:%d\r\n", SENSORID, msg.data.cValue[0]);
    	if (nrf_send_msg(&msg)){
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"DHT1, error send ok\r\n");
#endif
    	}
    	else{
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"DHT1, error send fail\r\n");
#endif
    	}
    }
    else{
        chprintf((BaseSequentialStream *)&SD2,"SENSOR:%d:DHT:1:TEMPERATURE:%.2f\r\n", SENSORID, (float)temperature/10);
        chprintf((BaseSequentialStream *)&SD2,"SENSOR:%d:DHT:1:HUMIDITY:%.2f\r\n", SENSORID, (float)humidity/10);
        // send temperature
    	msg.msgType = SENSOR_DATA;
		msg.valueType = TEMPERATURE;
		msg.data.iValue = temperature;
		if (nrf_send_msg(&msg)){
#if DEBUG
			chprintf((BaseSequentialStream *)&SD2,"DHT1, temperature send ok\r\n");
#endif
		}
		else{
#if DEBUG
			chprintf((BaseSequentialStream *)&SD2,"DHT1, temperature send fail\r\n");
#endif
		}
		// send humidity
		msg.valueType = HUMIDITY;
		msg.data.iValue = humidity;
		if (nrf_send_msg(&msg)){
#if DEBUG
			chprintf((BaseSequentialStream *)&SD2,"DHT1, humidity send ok\r\n");
#endif
		}
		else{
#if DEBUG
			chprintf((BaseSequentialStream *)&SD2,"DHT1, humidity send fail\r\n");
#endif
		}
    }

    // DHT_2
    readcnt = SENSOR_READ_MAX;
    read_ok = FALSE;
    while (readcnt-- > 0){
    	msg.data.cValue[0] = dht_read(2, &temperature, &humidity);
    	read_ok = msg.data.cValue[0] == DHT_NO_ERROR;
    	if (read_ok) break;
    }
	msg.msgType = SENSOR_ERROR;
	msg.sensorType = DHT;
	msg.owkey[0] = 2;
    if (!read_ok){
        chprintf((BaseSequentialStream *)&SD2,"ERROR:%d:DHT:2:%d\r\n", SENSORID, msg.data.cValue[0]);
    	if (nrf_send_msg(&msg)){
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"DHT2, error send ok\r\n");
#endif
    	}
    	else{
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"DHT2, error send fail\r\n");
#endif
    	}
    }
    else{
        chprintf((BaseSequentialStream *)&SD2,"SENSOR:%d:DHT:2:TEMPERATURE:%.2f\r\n", SENSORID, (float)temperature/10);
        chprintf((BaseSequentialStream *)&SD2,"SENSOR:%d:DHT:2:HUMIDITY:%.2f\r\n", SENSORID, (float)humidity/10);
        // send temperature
    	msg.msgType = SENSOR_DATA;
		msg.valueType = TEMPERATURE;
		msg.data.iValue = temperature;
		if (nrf_send_msg(&msg)){
#if DEBUG
			chprintf((BaseSequentialStream *)&SD2,"DHT2, temperature send ok\r\n");
#endif
		}
		else {
#if DEBUG
			chprintf((BaseSequentialStream *)&SD2,"DHT2, temperature send fail\r\n");
#endif
		}
		// send humidity
		msg.valueType = HUMIDITY;
		msg.data.iValue = humidity;
		if (nrf_send_msg(&msg)){
#if DEBUG
    		chprintf((BaseSequentialStream *)&SD2,"DHT2, humidity send ok\r\n");
#endif
		}
		else{
#if DEBUG
			chprintf((BaseSequentialStream *)&SD2,"DHT2, humidity send fail\r\n");
#endif
		}
    }

#if ADC
    // BATTERY
	readcnt = SENSOR_READ_MAX;
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

    	// other ADC
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
    }
#endif

    chEvtBroadcastFlags(&main_evsrc, (flagsmask_t)0);
    chThdSleepMilliseconds(2000);
  }
}
