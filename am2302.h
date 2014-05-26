#ifndef _AM2302_H_
#define _AM2302_H_
#ifdef __cplusplus
extern "C" {
#endif

#define DHT_COLLECTION_PERIOD_MS 2000UL
#define DHT_PRIO	(NORMALPRIO+1)

typedef enum {
	DHT_UNKNOWN,
	DHT_NO_ERROR,
	DHT_IRQ_TIMEOUT,
	DHT_TIMEOUT,
	DHT_RCV_TIMEOUT,
	DHT_DECODE_ERROR,
	DHT_CHECKSUM_ERROR,
} dht_error_t;

/*-----------------------------------------------------------------------------*/
void dht_init(void);
/* temperature in 1/10 deg C, humidity in 1/10 % */
dht_error_t dht_read(uint8_t channel, int *temperature, int *humidity);

#ifdef __cplusplus
}
#endif
#endif
