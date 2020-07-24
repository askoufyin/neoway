#include "service_parser.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"
#include "nmea_parser.h"
#include "main_task.h"
#include "eeprom.h"

extern volatile rcv_package_status_type_def service_rcv_package_status;
extern volatile transmit_package_status_type_def service_transmit_package_status;

extern UART_HandleTypeDef huart1;
extern uint8_t service_rxBuff[BUFF_LENGTH];
extern uint8_t service_txBuff[BUFF_LENGTH];

//UART N720
extern volatile rcv_package_status_type_def n720_rcv_package_status;
extern volatile transmit_package_status_type_def n720_transmit_package_status;

extern uint8_t n720_txBuff[BUFF_LENGTH];
extern uint8_t n720_rxBuff[BUFF_LENGTH];

static uint8_t ate = 1;
//---------------------------------------------------------------------------------

void echo(void)
{
	memcpy(service_txBuff, service_rxBuff, BUFF_LENGTH);
	service_uart_transmit_package_dma();
}

void at_error(void)
{
	sprintf((char*)service_txBuff, "ERROR\r");
	service_uart_transmit_package_dma();
	service_rcv_package_status = rcv_cplt;
	service_clear_rx();
}

void at_ok(void)
{
	sprintf((char*)service_txBuff, "OK\r");
	service_uart_transmit_package_dma();
	service_rcv_package_status = rcv_cplt;
	service_clear_rx();
}

uint8_t check_cmd(void)
{
	if(service_rcv_package_status == rcv_data_ready)
	{
		if(ate == 1)echo();
		uint16_t counter = 0;
		//ищем ATE
		while((service_rxBuff[counter] == 'A') && 
					(service_rxBuff[counter + 1] == 'T') &&
					(service_rxBuff[counter + 2] == 'E') &&
					(counter < 10))counter++;
		if(counter > 5)
		{
			char command = (char)service_rxBuff[counter + 3];
			switch(command)
			{
				case('0'):
				{
					ate = 0;
					at_ok();
					return 1;
				}
				case('1'):
				{
					ate = 1;
					at_ok();
					return 1;
				}
				default:
				{
					at_error();
					return 0;
				}
			}
		}
		counter = 0;
		//ищем '+'
		while((service_rxBuff[counter] != '+') && (counter < 10))counter++;
		if(counter > 8)
		{
			at_error();
			return 0;
		}else{
			at_ok();
		}
		if((service_rxBuff[counter - 2] == 'A') && 
			(service_rxBuff[counter - 1] == 'T'))
		{
			if((service_rxBuff[counter + 1] == 'G') &&
				 (service_rxBuff[counter + 2] == 'E') &&
				 (service_rxBuff[counter + 3] == 'T') &&
				 (service_rxBuff[counter + 4] == 'M') &&
				 (service_rxBuff[counter + 5] == 'I') &&
				 (service_rxBuff[counter + 6] == 'L'))
			{
				//получаем пробег
				uint32_t temp_mil = get_temp_total_mil();
				sprintf((char*)service_txBuff, "TOTAL_MIL_IS: %d\r\n", temp_mil);
				service_uart_transmit_package_dma();
				at_ok();
				return 1;
			}
			else if((service_rxBuff[counter + 1] == 'G') &&
							(service_rxBuff[counter + 2] == 'E') &&
							(service_rxBuff[counter + 3] == 'T') &&
							(service_rxBuff[counter + 4] == 'P') &&
							(service_rxBuff[counter + 5] == 'M') &&
							(service_rxBuff[counter + 6] == 'I') &&
							(service_rxBuff[counter + 7] == 'L'))
			{
				//получаем промежуточный пробег
				uint32_t temp_subtotal_mil = get_temp_subtotal_mil();
				sprintf((char*)service_txBuff, "SUBTOTAL_MIL_IS: %d\r\n", temp_subtotal_mil);
				at_ok();
				return 1;
			}
			else if((service_rxBuff[counter + 1] == 'S') &&
							(service_rxBuff[counter + 2] == 'E') &&
							(service_rxBuff[counter + 3] == 'T') &&
							(service_rxBuff[counter + 4] == 'M') &&
							(service_rxBuff[counter + 5] == 'I') &&
							(service_rxBuff[counter + 6] == 'L') &&
							(service_rxBuff[counter + 7] == '='))
			{
				counter += 8;//число начинается с 8 символа от знака +
				uint16_t command_pos = counter;
				char setmil[24] = {0,};
				while((counter++ != (24 + command_pos)) || (service_rxBuff[counter] != 0x00))
				{
					setmil[counter - command_pos] = service_rxBuff[counter];
				}
				//записываем пробег на флешку и в ОЗУ
				uint32_t mil = atoi(setmil);
				eeprom_write_totalmil(mil);
				at_ok();
				return 1;
			}
			else if((service_rxBuff[counter + 1] == 'S') &&
							(service_rxBuff[counter + 2] == 'E') &&
							(service_rxBuff[counter + 3] == 'T') &&
							(service_rxBuff[counter + 4] == 'P') &&
							(service_rxBuff[counter + 5] == 'M') &&
							(service_rxBuff[counter + 6] == 'I') &&
							(service_rxBuff[counter + 7] == 'L') &&
							(service_rxBuff[counter + 8] == '='))
			{
				counter += 8;//встаем на начало числа
				uint16_t command_pos = counter;//запоминаем позицию первого символа для коррекции
				char setpmil[24];
				while((counter++ != (24 + command_pos)) || (service_rxBuff[counter] != 0x00))
				{
					setpmil[counter - command_pos] = service_rxBuff[counter];
				}
				//записываем пробег на флешку и в ОЗУ
				uint32_t mil = atoi(setpmil);
				eeprom_write_subtotalmil(mil);
				return 1;
			}
		}
		else if((service_rxBuff[counter - 2] == 'A') && 
						(service_rxBuff[counter - 1] == 'D'))
		{
			//команды для неовея
			if((service_rxBuff[counter + 1] == 'G') && 
				 (service_rxBuff[counter + 2] == 'P') &&
				 (service_rxBuff[counter + 3] == 'S') &&
				 (service_rxBuff[counter + 4] == 'P') &&
				 (service_rxBuff[counter + 5] == 'O') &&
				 (service_rxBuff[counter + 6] == 'S'))
			{
				//запрашиваем геопзицию, отдаем в UART1
					//if(GGA_parser() == 1)
					//if(GPRMC_parser() == 1)
				if(gps_parser() == 1)
					{
						copy_n720_to_service();//копируем принятые от neoway данные в массив передачи сервисного uart
						service_uart_transmit_package_dma();
					}else{
						sprintf((char*)service_txBuff, "ERROR\r");
						service_uart_transmit_package_dma();
					}
			}
			else if((service_rxBuff[counter + 1] == 'S') &&
							(service_rxBuff[counter + 2] == 'H') &&
							(service_rxBuff[counter + 3] == 'U') &&
							(service_rxBuff[counter + 4] == 'T') &&
							(service_rxBuff[counter + 5] == 'D') &&
							(service_rxBuff[counter + 6] == 'O') &&
							(service_rxBuff[counter + 7] == 'W') &&
							(service_rxBuff[counter + 8] == 'N'))
			{
				n720_shutdown_cmd();
				at_ok();
				return 1;
			}
			else if((service_rxBuff[counter + 1] == 'P') &&
							(service_rxBuff[counter + 2] == 'O') &&
							(service_rxBuff[counter + 3] == 'W') &&
							(service_rxBuff[counter + 4] == 'E') &&
							(service_rxBuff[counter + 5] == 'R'))
			{
				//включаем неовей, проверяем связь
			}
			else if((service_rxBuff[counter + 1] == 'R') &&
							(service_rxBuff[counter + 2] == 'E') &&
							(service_rxBuff[counter + 3] == 'B') &&
							(service_rxBuff[counter + 4] == 'O') &&
							(service_rxBuff[counter + 5] == 'O') &&
							(service_rxBuff[counter + 6] == 'T'))
			{
				NVIC_SystemReset();
			}
		}
	}
	return 0;
}
