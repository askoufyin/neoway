#include "main.h"


typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

void accel_ini(void);
void accel_get_data(void);
void moving_callback(void);
