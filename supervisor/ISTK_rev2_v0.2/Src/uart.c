#include "uart.h"
#include "eeprom.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


//--------------------������ ��� ������ � �������� ��� neoway-------------------------
extern UART_HandleTypeDef huart2;

uint8_t n720_rxBuff[BUFF_LENGTH];
uint8_t n720_txBuff[BUFF_LENGTH];

uint8_t service_rxBuff[BUFF_LENGTH];
uint8_t service_txBuff[BUFF_LENGTH];

extern volatile uint8_t service_mode;

volatile uint16_t n720_rxBuff_counter = 0;

extern volatile uint8_t n720_on;

volatile rcv_package_status_type_def n720_rcv_package_status = rcv_cplt;
volatile transmit_package_status_type_def n720_transmit_package_status = transmit_cplt;
//------------------------------------------------------------------------------------

//---------------------������ � ����� ��� ���������� UART-----------------------------
extern UART_HandleTypeDef huart1;

uint8_t service_rxBuff[BUFF_LENGTH];
uint8_t service_txBuff[BUFF_LENGTH];

volatile uint16_t service_rxBuff_counter = 0;

volatile rcv_package_status_type_def service_rcv_package_status = rcv_cplt;
volatile transmit_package_status_type_def service_transmit_package_status = transmit_cplt;
//------------------------------------------------------------------------------------

//---------������� ������ ������------------
void n720_clear_rx(void)
{
	memset(n720_rxBuff, 0x00, BUFF_LENGTH);
}
void service_clear_rx(void)
{
	memset(service_rxBuff, 0x00, BUFF_LENGTH);
}
//-----------------------------------------

//---------������� ������ ��������---------
void n720_clear_tx(void)
{
	memset(n720_txBuff, 0x00, BUFF_LENGTH);
}
void service_clear_tx(void)
{
	memset(service_txBuff, 0x00, BUFF_LENGTH);
}
//----------------------------------------

//-----����������� ������ N720==>>service----

void copy_n720_to_service(void)
{
	for(uint16_t i = 0; i < (BUFF_LENGTH - 1); i++)
	{
		service_txBuff[i] = n720_rxBuff[i];
		if(n720_rxBuff[i] == 0x00)
		{
			service_txBuff[i] = 0x0D;//������ cr
			break;
		}
	}
}
//-------------------------------------------

//-------����������� ������-------
void transmit_error_handler(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		//while(1);
		HAL_UART_Transmit(&huart1, (uint8_t*)"UART1 tx error\r\n", sizeof("UART1 tx error\r\n"), 100);
		NVIC_SystemReset();
	}
	else if(huart == &huart2)
	{
		//while(1);
		HAL_UART_Transmit(&huart1, (uint8_t*)"UART2 tx error\r\n", sizeof("UART2 tx error\r\n"), 100);
		NVIC_SystemReset();
	}
	
}

void rcv_error_handler(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"UART1 rx error\r\n", sizeof("UART1 rx error\r\n"), 100);
		NVIC_SystemReset();
	}
	else if(huart == &huart2)
	{
		//while(1);
		HAL_UART_Transmit(&huart1, (uint8_t*)"UART2 rx error\r\n", sizeof("UART2 rx error\r\n"), 100);
		NVIC_SystemReset();
	}
}
//--------------------------------

//---------------------------------��������---------------------------------------
//--�������� ������
void n720_uart_transmit_package_dma(void)
{
	n720_transmit_package_status = transmit_busy;
	HAL_StatusTypeDef status;
	status = HAL_UART_Transmit_DMA(&huart2, n720_txBuff, strlen((char*)n720_txBuff));
	if(status == HAL_ERROR)transmit_error_handler(&huart2);
	while(huart2.hdmatx->State != HAL_DMA_STATE_READY);//����, ���� ���������� DMA
	while(n720_transmit_package_status != transmit_cplt);//���� ���� ��������� ��������
	n720_clear_tx();
}

//--�������� ������
void n720_uart_transmit_str_dma(char* str)
{
	n720_transmit_package_status = transmit_busy;
	HAL_StatusTypeDef status;
	status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)str, strlen(str));
	while(huart2.hdmatx->State != HAL_DMA_STATE_READY);//����, ���� ���������� DMA
	while(n720_transmit_package_status != transmit_cplt);
	if(status == HAL_ERROR)transmit_error_handler(&huart2);
}

void service_uart_transmit_package_dma(void)
{
	service_transmit_package_status = transmit_busy;
	HAL_StatusTypeDef status;
//	while(huart1.gState == HAL_UART_STATE_BUSY_TX);
	status = HAL_UART_Transmit_DMA(&huart1, service_txBuff, strlen((char*)service_txBuff));
	if(status == HAL_ERROR)transmit_error_handler(&huart1);//while(1);//transmit_error_handler(&huart1);
	while(huart1.hdmatx->State != HAL_DMA_STATE_READY);//����, ���� ���������� DMA
	while(service_transmit_package_status != transmit_cplt);//���� ���� ��������� ��������
	service_clear_tx();
}

void service_uart_transmit_str_dma(char* str)
{
	service_transmit_package_status = transmit_busy;
	HAL_StatusTypeDef status;
	status = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)str, strlen(str));
	while(huart1.hdmatx->State != HAL_DMA_STATE_READY);//����, ���� ���������� DMA
	while(service_transmit_package_status != transmit_cplt);
	if(status == HAL_ERROR)transmit_error_handler(&huart1);
}
//------------------------------------------------------------------------------

//-----------------------------------�����--------------------------------------
void n720_uart_rcv_package_it(void)
{
	HAL_StatusTypeDef status;
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);//��������� ��������� ���������� ��� ������
	status = HAL_UART_Receive_IT(&huart2, &n720_rxBuff[n720_rxBuff_counter], 1);
	if(status == HAL_ERROR)rcv_error_handler(&huart2);
}
void service_uart_rcv_package_it(void)
{
	HAL_StatusTypeDef status;
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);//��������� ��������� ���������� ��� ������
	status = HAL_UART_Receive_IT(&huart1, &service_rxBuff[service_rxBuff_counter], 1);
	if(status == HAL_ERROR)rcv_error_handler(&huart1);
}

void service_uart_rcv_pooling_init(void)
{
	HAL_UART_Receive_IT(&huart1, &service_rxBuff[service_rxBuff_counter++], 1);
}

//------------------------------------------------------------------------------


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		//!!!��� ������� � neoway
		if(((n720_rxBuff[n720_rxBuff_counter] == 0x0A) && (n720_rxBuff[n720_rxBuff_counter - 1] == 0x0D) && (n720_rxBuff[n720_rxBuff_counter - 2] == 'K') && 
				(n720_rxBuff[n720_rxBuff_counter - 3] == 'O')) || (n720_rxBuff_counter >= (BUFF_LENGTH - 1)))//��� ����������� CR+LF - ���������� �����
		
		//�������
		//if(((n720_rxBuff[n720_rxBuff_counter] == 0x0A) && (n720_rxBuff[n720_rxBuff_counter - 1] == 0x0D)) || (n720_rxBuff_counter >= (BUFF_LENGTH - 1)))
		{
			n720_rxBuff_counter = 0;
			n720_rcv_package_status = rcv_data_ready;
			HAL_UART_AbortReceive_IT(&huart2);//��������������
//			n720_uart_rcv_package_it();
		}else{
			if((n720_rxBuff[n720_rxBuff_counter] == ',') && ((n720_rxBuff[(n720_rxBuff_counter - 1)] == ',')))//��� ����������� ������������������ ,, - ��������� ��������(0x1F) ���������� ����� ��������
			{
				n720_rxBuff[n720_rxBuff_counter] = 0x1F;
				n720_rxBuff_counter++;
				n720_rxBuff[n720_rxBuff_counter] = ',';
			}
			n720_rxBuff_counter++;
			n720_rcv_package_status = rcv_busy;
			n720_uart_rcv_package_it();
		}
	}
	if(huart == &huart1)//��������� uart
	{
		//������ � ����������
		if(((service_rxBuff[service_rxBuff_counter] == 0x0A) && (service_rxBuff[service_rxBuff_counter - 1] == 0x0D)) || (service_rxBuff_counter == BUFF_LENGTH))
		{
			//����� ������ ��������
			service_rxBuff_counter = 0;
			service_rcv_package_status = rcv_data_ready;
//			HAL_UART_AbortReceive_IT(&huart1);//��������������
			service_uart_rcv_package_it();
		}else{//����� �� ��������, ���������� ��������� ��������� ������
			service_rxBuff_counter++;
			service_rcv_package_status = rcv_busy;
			service_uart_rcv_package_it();
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		n720_transmit_package_status = transmit_cplt;
	}
	if(huart == &huart1)
	{
		service_transmit_package_status = transmit_cplt;
	}
}

//-----------------������ ��������� ������------------------
void at_error(void)
{
	service_uart_transmit_str_dma("ERROR\r\n");
}

void at_ok(void)
{
	service_uart_transmit_str_dma("OK\r\n");
}

uint8_t ate = 1;
uint8_t service_parser(void)
{
	uint16_t counter;//������� ����, ������� ���������� ������� � ���� ������ ��� ������������������
	if(ate == 1)
	{
		memcpy(service_txBuff, service_rxBuff, (BUFF_LENGTH - 1));
		service_uart_transmit_package_dma();
	}

	counter = 0;
		//�������, �� ������ �� ATE
	while(!((service_rxBuff[counter] == 'A') &&
				(service_rxBuff[counter + 1] == 'T') &&
				(service_rxBuff[counter + 2] == 'E')) && 
				(counter < 7))counter++;
	/*
		���� ������ ������ 5 - ������� �� ������ �������, ���� ������ ��� ���������� ATE
		���� �� ������� ���� ������ 5, ������ ������� ������� ������ �������,
		���� ��������� ������� �� ATE, � ���� ������ ���������� ������ ������ ��� �������
	*/
	if(counter < 5)
	{
		switch(service_rxBuff[counter + 3])
		{
			case('1'):
				ate = 1;
				service_uart_transmit_str_dma("OK\r\n");
				return 1;
			case('0'):
				ate = 0;
				service_uart_transmit_str_dma("OK\r\n");
				return 1;
			default:
				service_uart_transmit_str_dma("ATE command not responded.\r\nYou must use ATE1 or ATE0 for enable/disable an echo.\r\n");
				return 0;
		}
	}
	counter = 0;
	while(!((service_rxBuff[counter] == 'A') &&
					(service_rxBuff[counter + 1] == 'T') &&
					(service_rxBuff[counter + 2] == '+')) && 
					(counter < 7))counter++;
	if(counter > 5)//���� �� ������� �� ������� ���� ������� ������
	{
		counter = 0;
		while(!((service_rxBuff[counter] == 'N') &&
						(service_rxBuff[counter + 1] == 'W') &&
						(service_rxBuff[counter + 2] == 'Y') &&
						(service_rxBuff[counter + 3] == '=')) && 
						(counter < 7))counter++;
		if(counter > 5)//���� ������� ��� ������ �� ������� 
		{
			service_uart_transmit_str_dma("Command not found.\r\nUse AT+HELP for getting list of commands.\r\n");
			return 0;
		}
		if(n720_on == 0)//���� n720 �� �������� - �������� �� ���� � �������� ��������
		{
			service_uart_transmit_str_dma("Error: N720 is not working.\r\n");
			return 0;
		}
		//����� �������, �������� NWY=, ��������� ������ ���� ������� ��� �������� �� �� neoway
		counter += 4;//��������� ��� NWY=
		for(; counter < BUFF_LENGTH; counter++)
		{
			service_rxBuff[counter - 4] = service_rxBuff[counter];//������� ������� NWY= �� �������
			if(service_rxBuff[counter] == 0x00)break;
		}
		service_uart_transmit_str_dma((char*)service_rxBuff);
		/*
			������� ������������� ������� �� n720_uart
		*/
		HAL_UART_AbortReceive_IT(&huart2);//��������� ������ ����� ������
		n720_uart_rcv_package_it();//��������� �����
		n720_uart_transmit_str_dma((char*)service_rxBuff);
		uint32_t tick = HAL_GetTick();
		
		while((HAL_GetTick() - tick) < 5000)//
		{
			if(n720_rcv_package_status == rcv_data_ready)break;
		}
		if (n720_rcv_package_status != rcv_data_ready)return 0;
		service_uart_transmit_str_dma((char*)n720_rxBuff);
		n720_clear_rx();
		return 1;

	}
	counter += 3;//������ �������
	if((service_rxBuff[counter] == 'R') && 
		(service_rxBuff[counter + 1] == 'E') &&
		(service_rxBuff[counter + 2] == 'A') &&
		(service_rxBuff[counter + 3] == 'D') &&
		(service_rxBuff[counter + 4] == 'T') &&
		(service_rxBuff[counter + 5] == 'O') &&
		(service_rxBuff[counter + 6] == 'T') &&
		(service_rxBuff[counter + 7] == 'A') &&
		(service_rxBuff[counter + 8] == 'L'))
	{
		sprintf((char*)service_txBuff, "TOTAL MIL IS: %d\r\n", eeprom_read_totalmil());
		service_uart_transmit_package_dma();
		eeprom_read_all();
		return 1;
	}
	else if((service_rxBuff[counter] == 'R') && 
			(service_rxBuff[counter + 1] == 'E') &&
			(service_rxBuff[counter + 2] == 'A') &&
			(service_rxBuff[counter + 3] == 'D') &&
			(service_rxBuff[counter + 4] == 'S') &&
			(service_rxBuff[counter + 5] == 'U') &&
			(service_rxBuff[counter + 6] == 'B') &&
			(service_rxBuff[counter + 7] == 'T') &&
			(service_rxBuff[counter + 8] == 'O') &&
			(service_rxBuff[counter + 9] == 'T') &&
			(service_rxBuff[counter + 10] == 'A') &&
			(service_rxBuff[counter + 11] == 'L'))
	{
		sprintf((char*)service_txBuff, "SUBTOTAL_MILEAGE_IS: %d\r\n", eeprom_read_subtotalmil());
		service_uart_transmit_package_dma();
		return 1;
	}
	else if((service_rxBuff[counter] == 'R') && 
			(service_rxBuff[counter + 1] == 'E') &&
			(service_rxBuff[counter + 2] == 'A') &&
			(service_rxBuff[counter + 3] == 'D') &&
			(service_rxBuff[counter + 4] == 'B') &&
			(service_rxBuff[counter + 5] == 'L') &&
			(service_rxBuff[counter + 6] == 'O') &&
			(service_rxBuff[counter + 7] == 'C') &&
			(service_rxBuff[counter + 8] == 'K'))
	{
		eeprom_send_block_to_uart();
		return 1;
	}
	else if((service_rxBuff[counter] == 'E') && 
			(service_rxBuff[counter + 1] == 'R') &&
			(service_rxBuff[counter + 2] == 'A') &&
			(service_rxBuff[counter + 3] == 'S') &&
			(service_rxBuff[counter + 4] == 'E'))
	{
		service_uart_transmit_str_dma("ERASE: IN PROGRESS...\r\n");
		eeprom_erase();
		service_uart_transmit_str_dma("ERASE: COMPLETE\r\n");
		//eeprom_write_block(125);
		return 1;
	}
	
	else if((service_rxBuff[counter] == 'W') && 
			(service_rxBuff[counter + 1] == 'R') &&
			(service_rxBuff[counter + 2] == 'I') &&
			(service_rxBuff[counter + 3] == 'T') &&
			(service_rxBuff[counter + 4] == 'E') &&
			(service_rxBuff[counter + 5] == 'T') &&
			(service_rxBuff[counter + 6] == 'O') &&
			(service_rxBuff[counter + 7] == 'T') &&
			(service_rxBuff[counter + 8] == 'A') &&
			(service_rxBuff[counter + 9] == 'L'))
	{
		//���� '=' ����� ������� ������ �� ������
		while(counter++)
		{
			if(service_rxBuff[counter] == '=')break;
			else if((service_rxBuff[counter] == 0x00) || (counter == (BUFF_LENGTH - 1)))
			{
				service_uart_transmit_str_dma("MILEAGE NOT FOUND\r\n");
				return 0;
				
			}
		}
		//����� '=', ��������� �����
		char arr_total_mil[32] = {0,};
		for(uint8_t i = 0; i < 32; i++ )
		{
			arr_total_mil[i] = service_rxBuff[++counter];
			if((service_rxBuff[counter] == 0x00) || (service_rxBuff[counter] == 0x0D) || (service_rxBuff[counter] == 0x0A) || (counter == (BUFF_LENGTH - 1)))break;
		}
		
		uint32_t total_mil = atoi(arr_total_mil);
		eeprom_write_totalmil(total_mil);
		sprintf((char*)service_txBuff, "WRITEMIL: %d\r\nOK\r\n", total_mil);
		service_uart_transmit_package_dma();
		eeprom_send_block_to_uart();
		return 1;
	}
	else if((service_rxBuff[counter] == 'W') && 
			(service_rxBuff[counter + 1] == 'R') &&
			(service_rxBuff[counter + 2] == 'I') &&
			(service_rxBuff[counter + 3] == 'T') &&
			(service_rxBuff[counter + 4] == 'E') &&
			(service_rxBuff[counter + 5] == 'S') &&
			(service_rxBuff[counter + 6] == 'U') &&
			(service_rxBuff[counter + 7] == 'B') &&
			(service_rxBuff[counter + 8] == 'T') &&
			(service_rxBuff[counter + 9] == 'O') &&
			(service_rxBuff[counter + 10] == 'T') &&
			(service_rxBuff[counter + 11] == 'A') &&
			(service_rxBuff[counter + 12] == 'L'))
	{
		//���� '=' ����� ������� ������ �� ������
		while(counter++)
		{
			if(service_rxBuff[counter] == '=')break;
			else if((service_rxBuff[counter] == 0x00) || (counter == (BUFF_LENGTH - 1)))
			{
				service_uart_transmit_str_dma("MILEAGE NOT FOUND.\r\n");
				return 0;
			}
		}
		//����� '=', ��������� �����
		char arr_subtotal_mil[32] = {0,};
		for(uint8_t i = 0; i < 32; i++ )
		{
			arr_subtotal_mil[i] = service_rxBuff[++counter];
			if((service_rxBuff[counter] == 0x00) || (service_rxBuff[counter] == 0x0D) || (service_rxBuff[counter] == 0x0A) || (counter == (BUFF_LENGTH - 1)))break;
		}
		
		uint32_t subtotal_mil = atoi(arr_subtotal_mil);
		eeprom_write_subtotalmil(subtotal_mil);
		sprintf((char*)service_rxBuff, "WRITEMIL: %d\r\nOK\r\n", subtotal_mil);
		service_uart_transmit_package_dma();
		eeprom_send_block_to_uart();
		return 1;

	}
	//����� ������� �������
	else if((service_rxBuff[counter] == 'R') && 
			(service_rxBuff[counter + 1] == 'E') &&
			(service_rxBuff[counter + 2] == 'S') &&
			(service_rxBuff[counter + 3] == 'E') &&
			(service_rxBuff[counter + 4] == 'T') &&
			(service_rxBuff[counter + 5] == 'T') &&
			(service_rxBuff[counter + 6] == 'O') &&
			(service_rxBuff[counter + 7] == 'T') &&
			(service_rxBuff[counter + 8] == 'A') &&
			(service_rxBuff[counter + 9] == 'L'))
	{
		service_uart_transmit_str_dma("OK\r\n");
		eeprom_write_totalmil(0x00U);
		eeprom_send_block_to_uart();
		return 1;
	}
	//����� ��������������  �������
	else if((service_rxBuff[counter] == 'R') && 
			(service_rxBuff[counter + 1] == 'E') &&
			(service_rxBuff[counter + 2] == 'S') &&
			(service_rxBuff[counter + 3] == 'E') &&
			(service_rxBuff[counter + 4] == 'T') &&
			(service_rxBuff[counter + 5] == 'S') &&
			(service_rxBuff[counter + 6] == 'U') &&
			(service_rxBuff[counter + 7] == 'B') &&
			(service_rxBuff[counter + 8] == 'T') &&
			(service_rxBuff[counter + 9] == 'O') &&
			(service_rxBuff[counter + 10] == 'T') &&
			(service_rxBuff[counter + 11] == 'A') &&
			(service_rxBuff[counter + 12] == 'L'))
	{
		service_uart_transmit_str_dma("OK\r\n");
		eeprom_write_subtotalmil(0x00U);
		eeprom_send_block_to_uart();
		return 1;
	}
	else if((service_rxBuff[counter] == 'R') &&
			(service_rxBuff[counter + 1] == 'E') &&
			(service_rxBuff[counter + 2] == 'B') &&
			(service_rxBuff[counter + 3] == 'O') &&
			(service_rxBuff[counter + 4] == 'O') &&
			(service_rxBuff[counter + 5] == 'T'))
	{
		NVIC_SystemReset();
		HAL_Delay(10);
		return 1;
	}
	else if((service_rxBuff[counter] == 'H') &&
			(service_rxBuff[counter + 1] == 'E') &&
			(service_rxBuff[counter + 2] == 'L') &&
			(service_rxBuff[counter + 3] == 'P'))
	{
		service_uart_transmit_str_dma("---------LIST OF COMMANDS:---------\r\nATE0 - disable echo mode\r\nATE1 - enable echo mode\r\nNWY= - send command to neoway\r\nAT+ERASE - erase EEPROM\r\nAT+WRITETOTAL= - write total mileage\r\nAT+WRITESUBTOTAL= - write subtotal mileage\r\nAT+READBLOCK - read mileage and subtotal mileage\r\nAT+REBOOT - reboot the device\r\n/---------------------------------/\r\n");
		return 1;
	}
	
	service_uart_transmit_str_dma("Command not responded.\r\nUse AT+HELP for getting list of commands.\r\n");
	return 0;
}

