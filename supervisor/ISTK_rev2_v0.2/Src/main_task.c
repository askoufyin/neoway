#include "main_task.h"
#include "uart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "nmea_parser.h"
#include "service_parser.h"
#include "gps_greatcircle.h"
#include "N720.h"
#include "eeprom.h"
#include "epoch_converter.h"
#include "indication.h"

extern volatile uint8_t n720_on;//������ ������� ��������� �� NEOWAY

extern uint8_t n720_rxBuff[BUFF_LENGTH];
extern uint8_t n720_txBuff[BUFF_LENGTH];

extern uint8_t service_rxBuff[BUFF_LENGTH];
extern uint8_t service_txBuff[BUFF_LENGTH];

extern volatile rcv_package_status_type_def n720_rcv_package_status;
extern volatile rcv_package_status_type_def service_rcv_package_status;
extern volatile transmit_package_status_type_def n720_transmit_package_status;

volatile uint8_t service_mode = 0;//0 - ����� ���������, ������� ������� 1 - ������ � ��������� UART

//UART neoway
extern UART_HandleTypeDef huart2;
//UART ���������
extern UART_HandleTypeDef huart1;
/*
	������ 6 ��� ���������� UART. ������� ����, ��� �������
	������� ������� ������������������ � ���������� ���������
	� ����� ������������� ��������� ������. ��� ������ ������
	��������� �� ���� ���������� ���������� ������������� ��������� 
	������ � ����������� � ����� �������� �������.
*/
extern TIM_HandleTypeDef htim2;//������ �������� �� ������ ��������� wdt � n720. ��� �� ���������� ����� ��������� ������������ n720
extern TIM_HandleTypeDef htim6;//������ ���������� ����������

//static uint32_t get_gps_timeout = 5000;//������� ������ ��������� (5 ���.)
/*
	write_counter - �������, ����������� ����������� ������ � eeprom ��� ���������� 
	������������� ��������. ������������ ��� ����, 
*/
//static uint32_t write_counter = 0;
static uint32_t total_mil = 0;
static uint32_t subtotal_mil = 0;
//static float temp_mil = 0;

void timeout_gps_pool(uint32_t timeout);


void main_task(void)
{

	//������ tx ����� rx
	print_run();
	service_print_run();
	
	eeprom_init();
	total_mil = eeprom_read_totalmil();
	subtotal_mil = eeprom_read_subtotalmil();
	
	//HAL_NVIC_DisableIRQ(TIM2_IRQn);
	HAL_TIM_Base_Start_IT(&htim2);//��������� ������ 
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);//��������� ������ ���������
	//!!
	service_uart_rcv_package_it();//��������� ����� �� ���������� UART �� ����� �������������
	//!!
	//N720_init();//���� ��� heartbeat ��������� neoway
	
	n720_ini();
	
	service_uart_transmit_str_dma("EEPROM INIT\r\nEEPROM READ\r\n");
	
	n720_uart_rcv_package_it();
//	service_uart_rcv_package_it();
	at_init();//�������������� �� ������� neoway
//	HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
	while(zero_point_init() != 1)//����������� ����� �������
	{
//		service_uart_transmit_str_dma((char*)n720_rxBuff);
		n720_clear_rx();
		n720_rcv_package_status = rcv_cplt;
		indication_handler(250);
		/*
			���� �������� �� ���������� UART
			� ������, ���� ������� �� ����������
			� ������������� �� ��������
		*/
		if(service_rcv_package_status == rcv_data_ready)//���� �������� ������ � ��������� UART - ������������ ��
		{
			service_parser();
			service_clear_rx();
			service_rcv_package_status = rcv_cplt;
		}
		if(huart1.RxState == HAL_UART_STATE_READY)service_uart_rcv_package_it();
	
		if(n720_on == 0)//��������� heartbeat
		{
			n720_ini();
		}
	}
//	HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);//��� ���������� ������������� ����� FAULT
	
	while(1)
	{
		indication_handler(250);
		if(GPRMC_parser() == 1)//����������� ����������
		{
			uint32_t temp_mil = get_distance();
			total_mil += temp_mil;//���� ������ ������� ��������� ������
			subtotal_mil += temp_mil;
			info_parser();//���������� ������ � neoway
			eeprom_write_mileage(total_mil, subtotal_mil);
			/*
				� ����������� �� �������� ������������ ������� ������ ���, ����� 
				������ � eeprom ������������� ����� ��� � 10 �����
			*/
//			uint32_t write_10_min = (60000 / get_gps_timeout) * 10;//������ � eeprom ��� � 10 �����
//			if(write_counter++ % write_10_min == 0)
//			{
//				eeprom_write_mileage(total_mil, subtotal_mil);
//				sprintf((char*)service_txBuff, "WRITE %d\r\n", total_mil);
//				service_uart_transmit_package_dma();
//			}
			
			HAL_Delay(1000);
//����������������			timeout_gps_pool(get_gps_timeout);//���������� ����� �������� ���������
		}
		n720_clear_rx();
		n720_rcv_package_status = rcv_cplt;
		n720_uart_rcv_package_it();
		
		if(service_rcv_package_status == rcv_data_ready)//���� �������� ������ � ��������� UART - ������������ ��
		{
			service_parser();
			service_clear_rx();
			service_rcv_package_status = rcv_cplt;
		}
		//if(huart1.RxState == HAL_UART_STATE_READY)service_uart_rcv_package_it();
		/*
			������� �� ������ �������� ���������, ���� neoway �����
			������������ ��������� � 	������������� ���.
		*/
		if(n720_on == 0)
		{
			HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);//�������� ��������� fault
			reboot_neoway();
			HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);//����� ��������� FAULT
		}
	}
}

//---------������� ������ ���������-----------
void timeout_gps_pool(uint32_t timeout)
{
	uint32_t tick = HAL_GetTick();
	while((HAL_GetTick() - tick) < timeout);//
}
//-------�������� ������ � ������� � ������ �����------
uint32_t get_temp_total_mil(void)
{
	return total_mil;
}

uint32_t get_temp_subtotal_mil(void)
{
	return subtotal_mil;
}
//---------���������� ������������� ������-----------
void reset_temp_subtotal_mil(void)
{
	subtotal_mil = 0;
}
//----------------------------------------------------
void print_run(void)
{
	sprintf((char*)n720_txBuff, "RUN_N720\r\n");
	n720_uart_transmit_package_dma();
}

void service_print_run(void)
{
	sprintf((char*)service_txBuff, "RUN_SERVICE\r\n");
	service_uart_transmit_package_dma();
}
