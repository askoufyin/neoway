#include "main.h"
#include <lsm6ds3_reg.h>

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;


void accel_ini(void);
float* accel_get_data(void);
void moving_callback(void);
