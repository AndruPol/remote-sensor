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

#define SENSORSNUM		0 		// ����� ������� ��������
#define SENSORSALL		1		// ����� ����� ��������
#define BATTSENSOR		0		// ����� ������� �������� ���������� ������� PB.00
#define ADC_TIMEOUT_MS	100		// �������� ������ ��������, ����

// �������� ���
#define BATTERYNORMA	995		// �������� "�����" ��� ������� > 3.1V
#define BATTERYLOW		500		// �������� ��� ����������� ������� ������ ������� < 3V

typedef enum {
	ADC_NO_ERROR,
	ADC_CONV_ERROR,
	ADC_TIMEOUT,
} adc_error_t;

// �������� �������
typedef struct _sensor sensor_t;
struct _sensor{
	uint8_t  type;		// ��� �������: 0 - �� ������������, 1 - ������, 2 - �������� �������
	uint16_t value;		// ��������� �������� ��������� ���
};

typedef struct _adc_read_t adc_read_t;
struct _adc_read_t {
	adc_error_t error; 				/* out */
	sensor_t sensor[SENSORSALL]; 	/* out */
};

#define ADC_PRIO	NORMALPRIO
extern adc_read_t sensors;			// �������� ��������

// ������������� ���
void sensors_init(void);
// ������ �������� ADC
adc_error_t sensors_read(void);

#ifdef __cplusplus
}
#endif
#endif /* SENSORS_H_ */
