/*
 *  DHT21/DHT22 driver
 */

#include "ch.h"
#include "hal.h"

#include "am2302.h"

#define DHT_BIT_TIMEOUT_US 	(80 * 4) /* one bit timeout */
#define DHT_START_PULSE_MS 	18
#define DHT_IRQ_TIMEOUT_MS 	2 		/* irq timeout */
#define DHT_PKT_SIZE 		5
#define DHT_PKT_TIMEOUT_MS 	10

#define DHT_CHANNEL1_GPIO		GPIOA /* channel 1 GPIO */
#define DHT_CHANNEL2_GPIO		GPIOB /* channel 2 GPIO */
#define DHT_CHANNEL1_PIN		15 /* channel 1 PIN */
#define DHT_CHANNEL2_PIN		3 /* channel 2 PIN */
#define DHT_CHANNEL1			ICU_CHANNEL_1
#define DHT_CHANNEL2			ICU_CHANNEL_2

#define ERROR_DIV 2
#define PERIOD_OK(x, l, h) \
	((x)->low >= ((l) - (l) / ERROR_DIV) && \
	(x)->low < ((l) + (l) / ERROR_DIV) && \
	((x)->period - (x)->low) >= ((h) - (h) / ERROR_DIV) && \
	((x)->period - (x)->low) < ((h) + (h) / ERROR_DIV))
/*-----------------------------------------------------------------------------*/

typedef struct _icu_capture_t icu_capture_t;
struct _icu_capture_t {
	uint16_t period;
	uint16_t low;
};

typedef struct _dht_read_t dht_read_t;
struct _dht_read_t {
	uint8_t		channel;		/* in */
	dht_error_t error; 			/* out */
	uint8_t data[DHT_PKT_SIZE]; /* out */
};

static BinarySemaphore icusem, cb_sem;
static volatile icu_capture_t icu_data;

static void icuwidthcb(ICUDriver *icup) {
  icu_data.low = icuGetWidth(icup);
}

static void icuperiodcb(ICUDriver *icup) {
  icu_data.period = icuGetPeriod(icup);
  chBSemSignalI(&cb_sem);
}

static void icuoverflowcb(ICUDriver *icup) {
  (void)icup;
  icu_data.period = 0;
  chBSemSignalI(&cb_sem);
}

static ICUConfig icucfgch1 = {
  ICU_INPUT_ACTIVE_LOW,
  1000000,                                    /* 1mHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  icuoverflowcb,
  DHT_CHANNEL1,
  0
};

static ICUConfig icucfgch2 = {
  ICU_INPUT_ACTIVE_LOW,
  1000000,                                    /* 1mHz ICU clock frequency.   */
  icuwidthcb,
  icuperiodcb,
  icuoverflowcb,
  DHT_CHANNEL2,
  0
};

/*
 * AM2302 read thread.
 */
static Thread *DHTThread_p;
static WORKING_AREA(waDHTThread, 128);
__attribute__((noreturn))
static msg_t DHTThread(void *arg) {

  (void)arg;
  chRegSetThreadName("DHTThd");

  chBSemInit(&cb_sem, TRUE);
  while (TRUE) {
	/* wait for read request */
	dht_read_t *req;
	Thread *tp;
	tp = chMsgWait();
	req = (dht_read_t *) chMsgGet(tp);
	chMsgRelease(tp, (msg_t) req);

	// set DHT pin low on 2ms
	if (req->channel == 1){
		palSetPadMode(DHT_CHANNEL1_GPIO, DHT_CHANNEL1_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
		palClearPad(DHT_CHANNEL1_GPIO, DHT_CHANNEL1_PIN);
	}
	else if (req->channel == 2){
		palSetPadMode(DHT_CHANNEL2_GPIO, DHT_CHANNEL2_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
		palClearPad(DHT_CHANNEL2_GPIO, DHT_CHANNEL2_PIN);
	}
	chThdSleepMilliseconds(DHT_START_PULSE_MS);

	if (req->channel == 1){
		icuStart(&ICUD2, &icucfgch1);
	}
	else if (req->channel == 2){
		icuStart(&ICUD2, &icucfgch2);
	}

	if (req->channel == 1){
		palSetPadMode(DHT_CHANNEL1_GPIO, DHT_CHANNEL1_PIN, PAL_MODE_INPUT);
	}
	else if (req->channel == 2){
		palSetPadMode(DHT_CHANNEL2_GPIO, DHT_CHANNEL2_PIN, PAL_MODE_INPUT);
	}
	icuEnable(&ICUD2);

	/* skip first falling edge */
	int i;
	// IRQ timeout or receive timeout
	if(chBSemWaitTimeout(&cb_sem, MS2ST(DHT_IRQ_TIMEOUT_MS)) == RDY_TIMEOUT) {
		req->error = DHT_IRQ_TIMEOUT;
		goto reply;
	}
	if(!icu_data.period) {
		req->error = DHT_TIMEOUT;
		goto reply;
	}

	/* start sequence received */
	if(!PERIOD_OK(&icu_data, 80, 80)) {
		req->error = DHT_DECODE_ERROR;
		goto reply;
	}

	for(i = 0; i < DHT_PKT_SIZE; i++) {
		unsigned int mask = 0x80;
		uint8_t byte = 0;
		while(mask) {
			if(chBSemWaitTimeout(&cb_sem, MS2ST(DHT_IRQ_TIMEOUT_MS)) == RDY_TIMEOUT) {
				req->error = DHT_IRQ_TIMEOUT;
				goto reply;
			}
			if(!icu_data.period) {
				req->error = DHT_TIMEOUT;
				goto reply;
			}

			/* next bit received */
			if(PERIOD_OK(&icu_data, 50, 70)) {
				byte |= mask; /* 1 */
			} else if(!PERIOD_OK(&icu_data, 50, 27)) {
				req->error = DHT_DECODE_ERROR;
				goto reply;
			}

			mask >>= 1;
		}
		req->data[i] = byte;
	}
	req->error = DHT_NO_ERROR;

reply:
	icuDisable(&ICUD2);
	icuStop(&ICUD2);
	chBSemSignal(&icusem);
  }
}

dht_error_t dht_read(uint8_t channel, int *temperature, int *humidity) {
	dht_read_t rd;
	dht_read_t *rd_p = &rd;

	rd.channel = channel;

	chBSemWait(&icusem); /* to be sure */

	chMsgSend(DHTThread_p, (msg_t) rd_p);

	/* wait for reply */
	if(chBSemWaitTimeout(&icusem, MS2ST(DHT_PKT_TIMEOUT_MS)) == RDY_TIMEOUT) {
		chBSemReset(&icusem, FALSE);
		return DHT_RCV_TIMEOUT;
	}
	chBSemReset(&icusem, FALSE);

	if(rd.error != DHT_NO_ERROR) {
		return rd.error;
	}

	/* compute checksum */
	unsigned int sum = 0;
	uint8_t i;
	for(i = 0; i < DHT_PKT_SIZE - 1; i++) sum += rd.data[i];
	if((sum & 0xff) != rd.data[i]) {
		return DHT_CHECKSUM_ERROR;
	}

	if (rd.data[1] == 0 && rd.data[3] == 0) {	// DHT11
		*humidity = ((unsigned int)rd.data[0]);
		*temperature = ((unsigned int)rd.data[2]);
	}
	else {	// DHT21/22
		/* read 16 bit humidity value */
		*humidity = ((unsigned int)rd.data[0] << 8) |
					(unsigned int)rd.data[1];

		/* read 16 bit temperature value */
		int val = ((unsigned int)rd.data[2] << 8) |
					(unsigned int)rd.data[3];
		*temperature = val & 0x8000 ? -(val & ~0x8000) : val;
	}
	return DHT_NO_ERROR;
}

void dht_init(void){
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_FULLREMAP;

	chBSemInit(&icusem, FALSE);
	DHTThread_p = chThdCreateStatic(waDHTThread, sizeof(waDHTThread), DHT_PRIO, DHTThread, NULL);
}
