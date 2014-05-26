/*
 * sensors.h
 */
#ifndef SENSORS_H_
#define SENSORS_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "ch.h"
#include "hal.h"

#define SENSORSNUM		0 		// число внешних датчиков
#define SENSORSALL		1		// общее число датчиков
#define BATTSENSOR		0		// номер датчика контроля напряжения батареи PB.00
#define ADC_TIMEOUT_MS	100		// задержка опроса датчиков, мсек

// значения АЦП
#define BATTERYNORMA	995		// значение "норма" для батареи > 3.1V
#define BATTERYLOW		500		// значение для обнаружения низкого заряда батареи < 3V

typedef enum {
	ADC_NO_ERROR,
	ADC_CONV_ERROR,
	ADC_TIMEOUT,
} adc_error_t;

// описание датчика
typedef struct _sensor sensor_t;
struct _sensor{
	uint8_t  type;		// тип датчика: 0 - не используется, 1 - датчик, 2 - контроль питания
	uint16_t value;		// последнее значение считанное АЦП
};

typedef struct _adc_read_t adc_read_t;
struct _adc_read_t {
	adc_error_t error; 				/* out */
	sensor_t sensor[SENSORSALL]; 	/* out */
};

#define ADC_PRIO	NORMALPRIO
extern adc_read_t sensors;			// описание датчиков

// инициализация АЦП
void sensors_init(void);
// чтение значений ADC
adc_error_t sensors_read(void);

#ifdef __cplusplus
}
#endif
#endif /* SENSORS_H_ */
