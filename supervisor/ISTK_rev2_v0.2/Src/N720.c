#include "N720.h"
#include "nmea_parser.h"
#include "uart.h"
#include "indication.h"

extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern volatile rcv_package_status_type_def service_rcv_package_status;

volatile uint8_t n720_on;
	//------------запускаем N720-----------------

void n720_error(void)
{
	
}

void stm32_deinit(void)
{
	//отключаем переферию
	HAL_UART_Abort_IT(&huart1);
	HAL_UART_Abort_IT(&huart2);
	HAL_UART_DeInit(&huart1);
	HAL_UART_DeInit(&huart2);
	HAL_DMA_Abort(&hdma_usart1_tx);
	HAL_DMA_DeInit(&hdma_usart1_tx);
	HAL_DMA_Abort(&hdma_usart2_tx);
	HAL_DMA_DeInit(&hdma_usart2_tx);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_DeInit(&htim2);
//	HAL_TIM_Base_Stop_IT(&htim6);
//	HAL_TIM_Base_DeInit(&htim6);
	HAL_SPI_Abort(&hspi1);
	HAL_SPI_DeInit(&hspi1);
}

//проверяем флаги 
void reboot_neoway(void)
{
	/*
		Отлкючаем таймер управления индикацией
		HAL_TIM_Base_Stop_IT(&htim6);
	*/
	N720_init();
}

void waiting(uint32_t timeout)
{
	uint32_t tick = HAL_GetTick();
	while((HAL_GetTick() - tick) < timeout)
	{
		IWDG->KR = 0x0000AAAAU;//сбрасываем вочдог
		if(service_rcv_package_status == rcv_data_ready)//если приходят данные в сервисный UART - обрабатываем их
		{
			service_parser();
			service_clear_rx();
			service_rcv_package_status = rcv_cplt;
			service_uart_rcv_package_it();
		}
		if(n720_on == 1)break;
	}
}

void stm32_sleep_mode(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;//вкл тактирование PWR
//SCB->SCR |= SCB_SCR_SLEEPDEEP; //для M3 разрешаем sleepdeep
	PWR->CR |= PWR_CR_PDDS;//выбираем режим Power Down Deepsleep
	PWR->CR |= PWR_CR_CWUF ; //очищаем wakeup flag
	PWR->CSR |= PWR_CSR_EWUP2;//разрешаем вэйкап, то есть пробуждение по переднему фронту на PC13
	HAL_PWR_EnterSTANDBYMode();
}

void N720_init(void)
{
	do
	{
		//HAL_GPIO_WritePin(MODEM_OFF_GPIO_Port, MODEM_OFF_Pin, GPIO_PIN_RESET);//начинаем перезагрузку
		HAL_GPIO_WritePin(N720_RESET_GPIO_Port, N720_RESET_Pin, GPIO_PIN_RESET);
		waiting(1000);//за это время reset должен быть в 0
		//HAL_GPIO_WritePin(N720_RESET_GPIO_Port, N720_RESET_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(MODEM_OFF_GPIO_Port, MODEM_OFF_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(N720_POWER_GPIO_Port, N720_POWER_Pin, GPIO_PIN_SET);//c rev3 не будет
		waiting(30000);//30000 - период загрузки модема
		service_uart_transmit_str_dma("N720: heartbeat is not detected.\r\nCheck PA0 pin connection on STM32.\r\n");
	}while(n720_on == 0);
}

void n720_ini_handler(void)
{
	static uint8_t switcher = 0;
	static uint32_t neoway_tick;
	switch(switcher)
	{
		case(0)://начинаем перезагрузку, кладем пины MODEM_OFF и N720_RESET
		{
			HAL_GPIO_WritePin(MODEM_OFF_GPIO_Port, MODEM_OFF_Pin, GPIO_PIN_RESET);//начинаем перезагрузку
			HAL_GPIO_WritePin(N720_RESET_GPIO_Port, N720_RESET_Pin, GPIO_PIN_RESET);
			if(HAL_GetTick() - neoway_tick > 1001)
			{
				switcher++;
				service_uart_transmit_str_dma("N720: reset.\r\n");
				neoway_tick = HAL_GetTick();
				
			}
			return;
		}
		case(1)://ждем 30 секунд загрузку
		{
			HAL_GPIO_WritePin(N720_RESET_GPIO_Port, N720_RESET_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MODEM_OFF_GPIO_Port, MODEM_OFF_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(N720_POWER_GPIO_Port, N720_POWER_Pin, GPIO_PIN_SET);//c rev3 не будет	
			
			if(HAL_GetTick() - neoway_tick > 30001)
			{
				switcher=0;
				service_uart_transmit_str_dma("N720: heartbeat is not detected.\r\nCheck PA0 pin connection on STM32.\r\n");
				neoway_tick = HAL_GetTick();
				
			}
			return;
		}
		default://если загрузка неудачна
		{
			switcher = 0;
			
			return;
		}
	}
}

void n720_ini(void)
{
	service_uart_transmit_str_dma("N720: heartbeat is lost, waiting for heartbeat.\r\n");
	while(n720_on == 0)
	{
		indication_handler(250);
		n720_ini_handler();
		
		if(service_rcv_package_status == rcv_data_ready)//если приходят данные в сервисный UART - обрабатываем их
		{
			service_parser();
			service_clear_rx();
			service_rcv_package_status = rcv_cplt;
			service_uart_rcv_package_it();
		}
	}
	service_uart_transmit_str_dma("N720: heartbeat OK.\r\n");
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/* Второй таймер отвечает за подсчет импульсов */
	if(htim == &htim2)
	{
		if(HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1) != 0x00)
		{
			//добавить опрос состояния шины питания 24 вольта
			((TIM2->CCR1) = 0x00);
			TIM2->CNT = 0x00;
			__HAL_TIM_SET_COUNTER(&htim2, 0x00);
			__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_UPDATE);
			__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);
			n720_on = 1;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)//если истекло время ожидания хардбит сигнала с N720
	{
		n720_on = 0;
	}
	
	
	if(htim == &htim6)//таймаут нахождения в сервисном режиме, как только он истекает - перезагружаем устройство и возвращаемся обратно в режим прослушивания спутников
	{
		HAL_TIM_Base_Stop_IT(&htim6);
		HAL_UART_Transmit(&huart1, (uint8_t*)"CONNECTION TIMEOUT\r\n", sizeof("CONNECTION TIMEOUT\r\n)"), 100);
		//service_uart_transmit_str_dma("CONNECTION TIMEOUT\r\n");
		HAL_NVIC_SystemReset();
	}
}

//void n720_shutdown(void)
//{
//	n720_shutdown_cmd();
//	HAL_GPIO_WritePin(MODEM_OFF_GPIO_Port, MODEM_OFF_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(N720_POWER_GPIO_Port, N720_POWER_Pin, GPIO_PIN_RESET);
//}


	//-------------------------------------------
