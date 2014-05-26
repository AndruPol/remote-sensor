/*
 * bh1750.h
 */

#ifndef BH1750_H_
#define BH1750_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "ch.h"
#include "hal.h"

typedef enum {
	BH1750_NO_ERROR,
	BH1750_ERROR,
	BH1750_TIMEOUT,
} bh1750_error_t;

typedef struct _bh1750_read_t bh1750_read_t;
struct _bh1750_read_t {
	bh1750_error_t error;	/* out */
	uint16_t value;		 	/* out */
};

void bh1750_init(void);
bh1750_error_t bh1750_read(int16_t *value);

#ifdef __cplusplus
}
#endif
#endif /* BH1750_H_ */
