/*
 * owtemp.h
 */
#ifndef OWTEMP_H_
#define OWTEMP_H_

#define OWDEVICES			6				// max 1-wire devices on bus
#define OWTEMP_PRIO		NORMALPRIO+1

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _ow_temp_t ow_temp_t;	// описание датчика температуры DS1820B
struct _ow_temp_t {
	uint8_t key[8];			// идентификатор датчика DS1820B
	float	value;			// значение датчика
};

typedef enum {
	OW_TEMP_UNKNOWN,
	OW_TEMP_NO_ERROR,
	OW_TEMP_NOT_FOUND,
	OW_TEMP_TIMEOUT,
} ow_temp_error_t;

typedef struct _ow_temp_read_t ow_temp_read_t;
struct _ow_temp_read_t {
	ow_temp_error_t error;
	ow_temp_t owtemp[OWDEVICES]; /* out */
};

extern ow_temp_read_t ow_temp_read;

void ow_temp_init(void);
ow_temp_error_t owtemp_read(void);

#ifdef __cplusplus
}
#endif
#endif /* OWTEMP_H_ */
