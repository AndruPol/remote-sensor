/*
 * main.h
 *
 *  Created on: 16.08.2013
 *      Author: pae
 */

#ifndef MAIN_H_
#define MAIN_H_
#ifdef __cplusplus
extern "C" {
#endif

#define	FIRMWARE		"0.4"	// версия прошивки
#define SLEEPTIME		720		// время режима пониженного энергопотребления, сек
#define SENSORID		0		// this sensor ID

typedef enum {
	SENSOR_INFO = 0,	// not implemented
	SENSOR_DATA,
	SENSOR_ERROR
} msgtype_t;

typedef enum {
	DS1820 = 0,
	BH1750,
	DHT,
	BMP085,
	ADC
} sensortype_t;

typedef enum {
	TEMPERATURE = 0,
	HUMIDITY,
	PRESSURE,
	LIGHT,
	VOLTAGE
} valuetype_t;

// описание формата посылки
#define MSGLEN		16
typedef struct MESSAGE MESSAGE_T;
struct MESSAGE{
	msgtype_t msgType;			// тип сообщения: 0 - инфо, 1 - значение датчика, 2 - ошибка
	uint8_t sensorID;			// board ID
	sensortype_t sensorType;	// тип датчика: 0 - DS1820, 1 - BH1750, 2 - DHT21/22 сенсор, 3 - BMP085, 4 - ADC
	valuetype_t valueType;		// тип значения: 0 - temperature, 1 - humidity, 2 - pressure, 3 - light, 4 - voltage
	uint8_t owkey[8];			// идентификатор датчика DS1820, номер датчика DHT в owkey[0]
	union	{					// значение датчика
		float	fValue;
		int32_t	iValue;
		uint8_t cValue[4];
	} data;
};

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */
