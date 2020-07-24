#include "lsm6ds3.h"
#include <string.h>
#include <stdio.h>

#define    BOOT_TIME   20 //ms
#define LSM6DS3_INT2_PIN GPIO_PIN_1
#define LSM6DS3_INT2_GPIO_PORT GPIOC
#define LSM6DS3_INT1_PIN GPIO_PIN_0
#define LSM6DS3_INT1_GPIO_PORT GPIOC
#define SENSOR_BUS hi2c1

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;

//static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
//static float acceleration_mg[3];
static float angular_rate_mdps[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
static uint8_t accel_flag = 1;
static stmdev_ctx_t dev_ctx;

static uint8_t moving = 0;

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

//write
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

//read
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

//transmit
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

//delay)
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

//-----------------------------------------------------------------------

/* Main Example --------------------------------------------------------------*/

void accel_ini(void)
{
  /* Initialize mems driver interface */
  
  lsm6ds3_int1_route_t int_1_reg;

  /* Uncomment if interrupt generation on DRDY INT2 pin. */
  //lsm6ds3_int2_route_t int_2_reg;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */

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
  lsm6ds3_reset_set(&dev_ctx, PROPERTY_ENABLE);//(0: continuous update; 1: output registers not updated until MSB and LSB have been read)
  do {
    lsm6ds3_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm6ds3_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  lsm6ds3_xl_full_scale_set(&dev_ctx, LSM6DS3_2g);
  lsm6ds3_gy_full_scale_set(&dev_ctx, LSM6DS3_2000dps);
	
	//включаем фильтр верхних частот гироскопа
	lsm6ds3_gy_hp_bandwidth_set(&dev_ctx, LSM6DS3_HP_CUT_OFF_2Hz07);
	
  /* Set Output Data Rate */
  lsm6ds3_xl_data_rate_set(&dev_ctx, LSM6DS3_XL_ODR_OFF);
  lsm6ds3_gy_data_rate_set(&dev_ctx, LSM6DS3_GY_ODR_12Hz5);

  /* Enable interrupt generation on DRDY INT1 pin */
  lsm6ds3_pin_int1_route_get(&dev_ctx, &int_1_reg);
  int_1_reg.int1_drdy_g = PROPERTY_ENABLE;
  int_1_reg.int1_drdy_xl = PROPERTY_ENABLE;
  lsm6ds3_pin_int1_route_set(&dev_ctx, &int_1_reg);

  /* Uncomment if interrupt generation routed on DRDY INT2 pin */
  //lsm6ds3_pin_int2_route_get(&dev_ctx, &int_2_reg);
  //int_2_reg.int2_drdy_g = PROPERTY_ENABLE;
  //int_2_reg.int2_drdy_xl = PROPERTY_ENABLE;
  //lsm6ds3_pin_int2_route_set(&dev_ctx, &int_2_reg);
}

//--------------get data----------------
float* accel_get_data(void)
{
	uint8_t reg;
	
	if(accel_flag)
	{
	  lsm6ds3_gy_flag_data_ready_get(&dev_ctx, &reg);
     if (reg)
     {
       /* Read gyroscope field data */
			memset(data_raw_angular_rate.u8bit, 0, 3 * sizeof(int16_t));
      lsm6ds3_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
       /* корректируем полученные данные [0] -3010 ; [1] +4900, [2] +1470, */
			angular_rate_mdps[0] = (lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]) -3010.0F);
      angular_rate_mdps[1] = (lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]) +4900.0F);
      angular_rate_mdps[2] = (lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]) +1470.0F);

      sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
			tx_com(tx_buffer, strlen((char const*)tx_buffer));
				//programm filter of data
			if((angular_rate_mdps[0] > 1000.0F) ||
				(angular_rate_mdps[1] > 1000.F) ||
				(angular_rate_mdps[2] > 1000.F))
			{
				//HAL_GPIO_TogglePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin);
				if(htim6.State == HAL_TIM_STATE_READY)
				{
					HAL_TIM_Base_Start_IT(&htim6);
				}
					
			}else{
				HAL_TIM_Base_Stop_IT(&htim6);
				TIM6->CNT = 0x00;//уточнить
			
				moving = 0;
			}
		}
	}
		if(moving == 1)HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
		return angular_rate_mdps;
}

//period elapsed callback &htim6
void moving_callback(void)
{
	moving = 1;
}

