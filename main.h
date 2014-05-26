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

#define	FIRMWARE		"0.2"	// версия прошивки
#define DELAYPERIOD		1000	// интервал секундного таймера задержки, мсек
#define SLEEPTIME		720		// время режима пониженного энергопотребления, сек
#define WORKTIME		5		// максимальное время работы в активном режиме, сек

// описание формата посылки
typedef struct MESSAGE MESSAGE_T;
struct MESSAGE{
	uint8_t msgType;	// тип сообщения: 0 - инфо, 1 - значение датчика, 2 - ошибка
	uint8_t errorCode;	// код ошибки для msgType == 0: 0 - нормальный заряд, 1 - низкий заряд батареи
	uint8_t sensorType;	// тип датчика: 0 - 1-wire, 1 - освещенности, 2 - DHT сенсор, 3 - батарея
	uint8_t valueType;	// тип значения: 0 - температура, 1 - влажность, 2 - значение АЦП
	uint8_t owkey[8];	// идентификатор датчика 1-wire, номер датчика для DHT в owkey[0]
	union	{			// значение датчика
		float	fValue;
		int32_t	iValue;
		uint8_t ch[4];
	} data;
};

void addr_hexstr(uint8_t *addr, uint8_t *str);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */
