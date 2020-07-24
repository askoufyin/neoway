/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ACCEL_SYSWKUP2_Pin GPIO_PIN_13
#define ACCEL_SYSWKUP2_GPIO_Port GPIOC
#define CS_EEPROM2_Pin GPIO_PIN_1
#define CS_EEPROM2_GPIO_Port GPIOA
#define CS_EEPROM1_Pin GPIO_PIN_4
#define CS_EEPROM1_GPIO_Port GPIOA
#define HOLD_Pin GPIO_PIN_0
#define HOLD_GPIO_Port GPIOB
#define WP_Pin GPIO_PIN_1
#define WP_GPIO_Port GPIOB
#define ACCEL_ADR_Pin GPIO_PIN_10
#define ACCEL_ADR_GPIO_Port GPIOB
#define BQ_CE_Pin GPIO_PIN_12
#define BQ_CE_GPIO_Port GPIOB
#define BQ_I2C2_SCL_Pin GPIO_PIN_13
#define BQ_I2C2_SCL_GPIO_Port GPIOB
#define BQ_I2C2_SDA_Pin GPIO_PIN_14
#define BQ_I2C2_SDA_GPIO_Port GPIOB
#define BQ_INTERRUPT_Pin GPIO_PIN_15
#define BQ_INTERRUPT_GPIO_Port GPIOB
#define BQ_INTERRUPT_EXTI_IRQn EXTI4_15_IRQn
#define N720_RESET_Pin GPIO_PIN_8
#define N720_RESET_GPIO_Port GPIOA
#define N720_POWER_Pin GPIO_PIN_9
#define N720_POWER_GPIO_Port GPIOA
#define MODEM_OFF_Pin GPIO_PIN_10
#define MODEM_OFF_GPIO_Port GPIOA
#define FAULT_LED_Pin GPIO_PIN_11
#define FAULT_LED_GPIO_Port GPIOA
#define CTRL_24V_Pin GPIO_PIN_12
#define CTRL_24V_GPIO_Port GPIOA
#define CTRL_24V_EXTI_IRQn EXTI4_15_IRQn
#define GPS_LED_Pin GPIO_PIN_15
#define GPS_LED_GPIO_Port GPIOA
#define POWER_LED_Pin GPIO_PIN_4
#define POWER_LED_GPIO_Port GPIOB
#define SERVICE_USART1_TX_Pin GPIO_PIN_6
#define SERVICE_USART1_TX_GPIO_Port GPIOB
#define SERVICE_USART1_RX_Pin GPIO_PIN_7
#define SERVICE_USART1_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
