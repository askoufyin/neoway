#include "lsm6ds3_pooling.h"
#include <string.h>
#include <stdio.h>
#include <lsm6ds3_reg.h>

#define    BOOT_TIME   20 //ms
#define SENSOR_BUS hi2c1

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_angular_rate;
static float angular_rate_mdps[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/*   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
stmdev_ctx_t dev_ctx;

void accel_init(void)
{
	/* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID */
  lsm6ds3_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LSM6DS3_ID)
    while(1)
    {
      /* manage here device not found */
    }

  /* Restore default configuration */
  lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6ds3_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*  Enable Block Data Update */
  lsm6ds3_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  lsm6ds3_xl_full_scale_set(&dev_ctx, LSM6DS3_2g);
  lsm6ds3_gy_full_scale_set(&dev_ctx, LSM6DS3_2000dps);

  /* Set Output Data Rate for Acc and Gyro */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_12Hz5);
  lsm6ds3_gy_data_rate_set(&dev_ctx, LSM6DS3_GY_ODR_12Hz5);

  /* Read samples in polling mode (no int) */
}

void gyro_get_data(void)
{		
	uint8_t reg;
  
	lsm6ds3_gy_flag_data_ready_get(&dev_ctx, &reg);
    if (reg)
    {
      /* Read angular rate field data */
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
      lsm6ds3_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
      angular_rate_mdps[0] =
        lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] =
        lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] =
        lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

      sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0],
              angular_rate_mdps[1],
              angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }
}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DS3_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Read(handle, LSM6DS3_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
  return 0;
}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

