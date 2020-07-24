#ifndef __LSM6DS3TR_C_H
#define __LSM6DS3TR_C_H

#include "main.h"

#define ACC_BUFFER_SIZE   90

extern uint8_t acc_new;

void acc_init(void);
int acc_update(void);

void acc_get_data(float *data);
void acc_get_pattern(uint8_t *pattern);

#endif /* __LSM6DS3TR_C_H */
