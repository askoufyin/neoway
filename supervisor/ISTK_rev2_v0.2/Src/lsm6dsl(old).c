#include "lsm6dsl.h"

#include "lsm6ds3tr_c_reg.h"

#include <string.h>

extern I2C_HandleTypeDef hi2c1;

static uint8_t AccPattern[ACC_BUFFER_SIZE];
static float AccData[ACC_BUFFER_SIZE];

uint8_t acc_new = 0;



stmdev_ctx_t dev_ctx;

static int32_t lsm6dsl_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len) {
  if (handle == &hi2c1)
  {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle, LSM6DS3TR_C_I2C_ADD_L, Reg,
      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);

    return status;
  }

  return HAL_ERROR;
}

static int32_t lsm6dsl_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len) {
  if (handle == &hi2c1) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, LSM6DS3TR_C_I2C_ADD_L, Reg,
    I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);

    return status;
  }

  return HAL_ERROR;
}

void acc_init(void) {
  uint8_t whoamI, rst;

  dev_ctx.write_reg = lsm6dsl_write;
  dev_ctx.read_reg = lsm6dsl_read;
  dev_ctx.handle = &hi2c1;

  /*
   *  Check device ID
   */
  whoamI = 0;
  lsm6ds3tr_c_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != 0x69)//LSM6DS3TR_C_ID)
    while(1); /*manage here device not found */

  /*
   *  Restore default configuration
   */
  lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
  } while (rst);

  ///

  /*
   *  Enable Block Data Update (output registers not updated until MSB and LSB have been read) 
   */
  lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set output data rate
   */
  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_OFF);
  //lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_OFF); turned on
	lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_12Hz5);

  /*
   * Set full scale
   */
  lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);
	
	lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, LSM6DS3TR_C_125dps);
  /*
   * Configure analog filtering chain (no aux interface)
   */
  lsm6ds3tr_c_xl_filter_analog_set(&dev_ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz);

  /*
   * Accelerometer digital low pass filter - LPF1 + LPF2 path 
   */
  lsm6ds3tr_c_xl_lp2_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LP_NA);
  /*
   *  Enable FIFO mode 
   */
  lsm6ds3tr_c_fifo_data_rate_set(&dev_ctx, LSM6DS3TR_C_FIFO_12Hz5);
	
  /*
   *  No decimation in FIFO (all value copy to FIFO)
   */
  //lsm6ds3tr_c_fifo_xl_batch_set(&dev_ctx, LSM6DS3TR_C_FIFO_XL_NO_DEC);
	
	lsm6ds3tr_c_fifo_gy_batch_set(&dev_ctx, LSM6DS3TR_C_FIFO_GY_NO_DEC);
  /*
   *  Disable gyroscope in FIFO
   */
  //lsm6ds3tr_c_fifo_gy_batch_set(&dev_ctx, LSM6DS3TR_C_FIFO_GY_DISABLE);
	lsm6ds3tr_c_fifo_xl_batch_set(&dev_ctx, LSM6DS3TR_C_FIFO_XL_DISABLE);

  /*
   *  Set FIFO watermark threshold
   */
  lsm6ds3tr_c_fifo_watermark_set(&dev_ctx, ACC_BUFFER_SIZE);

  /*
   *  FIFO stop values memorization at threshold level
   */
  lsm6ds3tr_c_fifo_stop_on_wtm_set(&dev_ctx, 1);
	
  /*
   *  Activate INT2 on FIFO overrun
   */
  lsm6ds3tr_c_int2_route_t interr;
  memset(&interr, 0, sizeof(interr));
  interr.int2_fifo_ovr = 1;

  lsm6ds3tr_c_pin_int2_route_set(&dev_ctx, interr);

  /*
   *  Continuous mode
   */
  lsm6ds3tr_c_fifo_mode_set(&dev_ctx, LSM6DS3TR_C_STREAM_MODE);

  ///

  /*
   *  Set threshold for wakeup
   */
  //lsm6ds3tr_c_wkup_threshold_set(&dev_ctx, 1);

  /*
   *  Set wake up duration
   */
  //lsm6ds3tr_c_wkup_dur_set(&dev_ctx, 5);//длительность wakeup импульса. Задаем длинну импульса для того, чтобы при вейкапе отслеживать состояние вейкап пина

  /*
   *  Activate INT1 for wake-up
   */
  lsm6ds3tr_c_int1_route_t interrw;
  memset(&interrw, 0, sizeof(interrw));
  interrw.int1_wu = 1;

  lsm6ds3tr_c_pin_int1_route_set(&dev_ctx, interrw);
}

//int acc_update(void) {
//  if (acc_new == 1) {
//    int i;

//    for(i = 0; i < ACC_BUFFER_SIZE; i++) {
//      AccData[i] = 0;
//      AccPattern[i] = 0;
//    }

//    for(i = 0; i < ACC_BUFFER_SIZE; i++) {
//      static uint16_t pattern;
//      lsm6ds3tr_c_fifo_pattern_get(&dev_ctx, &pattern);

//      uint8_t acc[2];
//      lsm6ds3tr_c_fifo_raw_data_get(&dev_ctx, acc, 2);

//      int16_t a = (acc[1] << 8) | acc[0];
//      float b = LSM6DSL_FROM_FS_2g_TO_mg(a);

//      AccData[i] = b;

//      // pattern 0 - X, 1 - Y, 2 - Z
//      AccPattern[i] = (uint8_t) pattern;
//    }

//    lsm6ds3tr_c_fifo_mode_set(&dev_ctx, LSM6DS3TR_C_BYPASS_MODE); // Clear FIFO
//    lsm6ds3tr_c_fifo_mode_set(&dev_ctx, LSM6DS3TR_C_STREAM_MODE);

//    acc_new = 0;
//    
//    return 1;
//  }
//  
//  return 0;
//}

void acc_get_data(float *data) {
  int i;
  for(i = 0; i < ACC_BUFFER_SIZE; i++) {
    data[i] = AccData[i];
  }
}

void acc_get_pattern(uint8_t *pattern) {
  int i;
  for(i = 0; i < ACC_BUFFER_SIZE; i++) {
    pattern[i] = AccPattern[i];
  }
}

