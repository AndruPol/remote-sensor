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

#define	FIRMWARE		"0.2"	// ������ ��������
#define DELAYPERIOD		1000	// �������� ���������� ������� ��������, ����
#define SLEEPTIME		720		// ����� ������ ����������� �����������������, ���
#define WORKTIME		5		// ������������ ����� ������ � �������� ������, ���

// �������� ������� �������
typedef struct MESSAGE MESSAGE_T;
struct MESSAGE{
	uint8_t msgType;	// ��� ���������: 0 - ����, 1 - �������� �������, 2 - ������
	uint8_t errorCode;	// ��� ������ ��� msgType == 0: 0 - ���������� �����, 1 - ������ ����� �������
	uint8_t sensorType;	// ��� �������: 0 - 1-wire, 1 - ������������, 2 - DHT ������, 3 - �������
	uint8_t valueType;	// ��� ��������: 0 - �����������, 1 - ���������, 2 - �������� ���
	uint8_t owkey[8];	// ������������� ������� 1-wire, ����� ������� ��� DHT � owkey[0]
	union	{			// �������� �������
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
