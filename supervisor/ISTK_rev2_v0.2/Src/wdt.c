//#include "wdt.h"
//#include <string.h>
//#include <stdio.h>
//#include "lsm6ds3_pooling.h"
//#include "uart.h"

//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim6;
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//volatile uint8_t n720_on = 0;//флажок наличия импульсов от NEOWAY

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	/* Второй таймер отвечает за подсчет импульсов */
//	if(htim == &htim2)
//	{
//		if(HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1) != 0x00)
//		{
//			//добавить опрос состояния шины питания 24 вольта
//			((TIM2->CCR1) = 0x00);
//			TIM2->CNT = 0x00;
//			__HAL_TIM_SET_COUNTER(&htim2, 0x00);
//			__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_UPDATE);
//			__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);
//			n720_on = 1;
//		}
//	}
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim == &htim2)//если истекло время ожидания хардбит сигнала с N720
//	{
//		n720_on = 0;
//	}
//	
//	
//	if(htim == &htim6)//таймаут нахождения в сервисном режиме, как только он истекает - перезагружаем устройство и возвращаемся обратно в режим прослушивания спутников
//	{
//		HAL_TIM_Base_Stop_IT(&htim6);
//		HAL_UART_Transmit(&huart1, (uint8_t*)"CONNECTION TIMEOUT\r\n", sizeof("CONNECTION TIMEOUT\r\n)"), 100);
//		//service_uart_transmit_str_dma("CONNECTION TIMEOUT\r\n");
//		HAL_NVIC_SystemReset();
//	}
//}



