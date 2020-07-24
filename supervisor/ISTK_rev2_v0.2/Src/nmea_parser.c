#include "nmea_parser.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart.h"
#include "main_task.h"
#include "eeprom.h"
#include "epoch_converter.h"

extern uint8_t service_txBuff[BUFF_LENGTH];

extern uint8_t n720_rxBuff[BUFF_LENGTH];
extern uint8_t n720_txBuff[BUFF_LENGTH];
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;//таймер индикации отсутствия сим карты

extern volatile rcv_package_status_type_def n720_rcv_package_status;

gpgga_struct_type_def gpgga_struct;//struct for gpgga package
gprmc_struct_type_def gprmc_struct;//struct 4 gprmc pack
gps_struct_typedef gps_struct;

//uint8_t checkSum(uint16_t start_message_index);
//void set_float(void);

uint8_t fix_type;

uint8_t sim_error = 0;//ошибка симкарты

//контрольная сумма высчитывается как XOR всех байт между символами $ и *
/*
Конец тела сообщения обозначается символом «*». 
Далее следует контрольная сумма всех символов, 
содержащихся между «$» и «*», то есть всего тела сообщения, 
включая и сами данные и идентификаторы в начале сообщения. 
Контрольная сумма вычисляется как XOR (исключающее ИЛИ) 
всех шестнадцатеричных кодов ASCII символов сообщения.
*/
uint8_t checkSum(uint16_t start_message_index)
{
	uint8_t message_checksum = 0, checksum = 0;
	//сначала прочитаем контрольную сумму полученного сообщения
	uint16_t counter_index = start_message_index + 1;//идем в тело сообщения, следующий символ вправо от $
	while(n720_rxBuff[counter_index] != '*')//считаем контрольную сумму байт, находящихся между символами начала сообщения ($) и конца сообщения (*)
	{
		//если встречаем последовательность ,0х1F, - не учитываем 0x1F при расчете заменяя на 0х30
		if((n720_rxBuff[counter_index] == ',') && (n720_rxBuff[counter_index+1] == 0x1F) && (n720_rxBuff[counter_index+2] == ','))
		{
			checksum^=n720_rxBuff[counter_index];
			++counter_index;
			n720_rxBuff[counter_index] = '0';//символ 0x1F переназначаем нулем, но не учитываем при подсчете контрольной суммы
			//++counter_index;
			//checksum^=n720_rxBuff[counter_index];
		}else{			
			checksum^=n720_rxBuff[counter_index];//расчет контрольной суммы производится операцией XOR каждого последующего байта N к предыдущему
		}
		if(++counter_index == BUFF_LENGTH)return 0;//если прочитали весь буфер, но не нашли разделителя контрольной суммы(*) возвращаем ошибку
	}	
	/*
	теперь переводим из ASCII в HEX контрольную сумму, которую получили в сообщении
	числа от 0 до 9 находятся в ASCII под 0x30 - 0x39 соответственно
	поэтому для получения чистого числа будем отнимать 0x30
	для получения чисел с А по F отнимаем 0х37
	Диапазон чисел 0-9 меньше 0х40, диапазон A-F больше 0х40
	*/
	if(n720_rxBuff[++counter_index] > 0x40) message_checksum = (n720_rxBuff[counter_index] - 0x37) << 4;
	else message_checksum = (n720_rxBuff[counter_index] - 0x30) << 4;
	if(n720_rxBuff[++counter_index] > 0x40) message_checksum += (n720_rxBuff[counter_index] - 0x37);
	else message_checksum += (n720_rxBuff[counter_index] - 0x30);
	//сравниваем прочитанную и вычисленную контрольные суммы
	sprintf((char*)n720_txBuff, "message_crc %0x\r", message_checksum);
	n720_uart_transmit_package_dma();
	

	sprintf((char*)n720_txBuff, "checksum_crc %0x\r", checksum);
	n720_uart_transmit_package_dma();
	
	if(message_checksum != checksum)return 0;
	return 1;
}


//добавляем точку после градусов в строчный массив, и убираем точку после минут
void set_float_gprmc(void)
{
	uint8_t arr_size;
	arr_size = (sizeof(gprmc_struct.latitude)-1);
	uint8_t i = arr_size;
	for(; i >= 2; i--)
	{
		if(gprmc_struct.latitude[i] == 0x00)continue;
		gprmc_struct.latitude[i + 1] = gprmc_struct.latitude[i];
		if(i == 2)gprmc_struct.latitude[i] = '.';
	}
	//ищем вторую точку, которую требуется очистить 
	i = arr_size;
	for(; i >= 2; i--)
	{
		if(gprmc_struct.latitude[i] == 0x00)continue;
		if(gprmc_struct.latitude[i] == '.')break;
	}
	for(; i < arr_size; i++)
	{
		if(i == 2)break;//если в предыдущем цикле нашли точку после градусов - выходим
		gprmc_struct.latitude[i] = gprmc_struct.latitude[i + 1];
		if(gprmc_struct.latitude[i] == 0x00)break;//выход из пустого массива
	}
	//для второго массива
	arr_size = (sizeof(gprmc_struct.longitude)-1);
	i = arr_size;
	for(; i >= 3; i--)
	{
		if(gprmc_struct.longitude[i] == 0x00)continue;
		else gprmc_struct.longitude[i + 1] = gprmc_struct.longitude[i];
		if(i == 3)gprmc_struct.longitude[i] = '.';
	}
	//ищем вторую точку, которую требуется очистить 
	i = arr_size;
	for(; i >= 3; i--)
	{
		if(gprmc_struct.longitude[i] == 0x00)continue;
		if(gprmc_struct.longitude[i] == '.')break;
	}
	for(; i < arr_size; i++)
	{
		if(i == 3)break;
		gprmc_struct.longitude[i] = gprmc_struct.longitude[i + 1];
		if(gprmc_struct.longitude[i] == 0x00)break;
	}
}

float knots_to_kmh(void)
{
	float kmh;
	kmh = atof(gprmc_struct.speed_over_ground_knots);
	memset(gprmc_struct.speed_over_ground_knots, 0x00, sizeof(gprmc_struct.speed_over_ground_knots));
	kmh *= 1.852;
	sprintf(gprmc_struct.speed_over_ground_knots, "%f", kmh);
	return kmh;
}

uint8_t n720_ack_timeout(uint32_t timeout)
{
	uint32_t tick = HAL_GetTick();
	while((HAL_GetTick() - tick) < timeout)//
	{
		if(n720_rcv_package_status == rcv_data_ready)return 1;
	}
	return 0;
}



//даем команду на отключение, принимаем ответ
uint8_t n720_off_cmd(void)
{
	sprintf((char*)n720_txBuff, "AT+SHUTDOWN\r");
	n720_uart_transmit_package_dma();
	if(n720_ack_timeout(10000) == 0)return 0;
	
	uint16_t counter = 0;
	while(!((n720_rxBuff[counter] == 'O') &&
					(n720_rxBuff[counter + 1] == 'F') &&
					(n720_rxBuff[counter + 2] == 'F')) &&
					(counter < 8))counter++;
	if(counter > 5)return 0;
	else return 1;
}

void n720_shutdown_cmd(void)
{
	uint8_t counter = 0, status = 0;
	//сохраняем пробег в eeprom
	eeprom_write_totalmil(get_temp_total_mil());
	eeprom_write_subtotalmil(get_temp_subtotal_mil());
	do
	{
		status = n720_off_cmd();
	}while((counter++ < 3) || (status != 1));
	if(status == 0)
	{
		sprintf((char*)service_txBuff, "AT COMMANDS: SLEEP_CMD\r\nNEOWAY USART2 NOT RESPONDED\r\nFORCED REBOOT\r\n");
		service_uart_transmit_package_dma();
	}
	//снимаем питание с neoway
	HAL_GPIO_WritePin(MODEM_OFF_GPIO_Port, MODEM_OFF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(N720_POWER_GPIO_Port, N720_POWER_Pin, GPIO_PIN_RESET);
}

//---передача пробега, запрос состояния sim---
uint8_t info_parser(void)
{
	sprintf((char*)n720_txBuff, "INFO:%d,%d,NO_BATTERY\r\n", get_temp_total_mil(), get_temp_subtotal_mil());//формируем пакет !добавить процент заряда батареи в версии с батарейным питанием 
	n720_uart_transmit_package_dma();
	if(n720_ack_timeout(2000) == 0)//ждем ответ 2 сек
	{
		service_uart_transmit_str_dma("TIMEOUT\r\n");
		n720_rcv_package_status = rcv_cplt;
		n720_clear_rx();
		return 0;
	}
	/*
		Когда ответ пришел - разбираем его
		В ответе получаем статус сим карты 
	*/
	uint16_t start_pos = 0;
	while(!((n720_rxBuff[start_pos] == '$')&&
					(n720_rxBuff[start_pos + 1] == 'I')&&
					(n720_rxBuff[start_pos + 2] == 'N')&&
					(n720_rxBuff[start_pos + 3] == 'F')&&
					(n720_rxBuff[start_pos + 4] == 'O'))&&
					(start_pos < 25))start_pos++;
	if(start_pos > 20)return 0;//первые 4 символа(NMEA), пятый должен быть($) если пакет смещен в буфере отказваемся от него
	
	/*\
	|=|		принимаемый пакет $INFO,1,2*crc8
	+|+		1 - информация о статусе SIM V - valid, I - invalid
	|=|		2 - нужно ли сбрасывать промежуточный пробег 1 - нужно, 2 - не нужно
	\*/
	
	if(checkSum(start_pos) == 0)return 0;//если контрольная сумма не совпадает отказываемя от пакета
	
	uint8_t field_counter = 0;//счетчик запятых, полей пакета
	for(uint16_t i = start_pos + 5; i < BUFF_LENGTH; i++)
	{
		if(n720_rxBuff[start_pos++] == ',')field_counter++;
		switch (field_counter)
		{
			case(1)://получаем информацию о статусе GSM - V - valid, I - invalid
			{
				if(n720_rxBuff[start_pos] == 'V')
				{
					sim_error = 0;
					if(htim6.State != HAL_TIM_STATE_READY)//если таймер запущен - останавливаем его
					{
						HAL_TIM_Base_Stop_IT(&htim6);//увидели наличие СИМ, останавливаем таймер
						TIM6->CNT = 0;//сбрасываем счетчик
						HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);//возвращием светодиод в исходное положение
					}
				}
				else if(n720_rxBuff[start_pos] == 'I')
				{
					sim_error = 1;
					if(htim6.State == HAL_TIM_STATE_READY)
					{
						HAL_TIM_Base_Start_IT(&htim6);//заводим таймер моргалку светодиодом
					}
				}
				else return 0;
				start_pos++;
				break;
			}
			case(2)://смотрим флажок промежуточного пробега
			{
				if(n720_rxBuff[start_pos++] == '1')reset_temp_subtotal_mil();
				break;
			}
		}
		if(n720_rxBuff[start_pos] == 0x00)break;
	}
	return 1;
}

//-----------запрос nmea пакета с датой и временем---------------
/*
	Возвращаем 0 если потеря пакета или данные невалидны (смещение пакета, неверная контрольная сумма)
	Возвращаем 1 если пакет цел, позиция валидна и движение распозано(скорость больше 0)
	Возвращаем 2 если пакет цел, позиция валидна, но нет движения (скорость 0)
*/
uint8_t GPRMC_parser(void)
{
	uint8_t field_pos = 0;//счетчик полей сообщения по разделителям ','
	uint8_t field_symbol_pos = 0;//счетчик символов в каждом поле струтуры
	uint16_t start_pos = 0, i = 0;
	
	//запрашиваем пакет
	sprintf((char*)n720_txBuff, "AT$MYGPSPOS=3\r");//отправляем запрос
	n720_uart_transmit_package_dma();
	if(n720_ack_timeout(2000) == 0)//таймаут ответа
	{
		service_uart_transmit_str_dma("TIMEOUT\r\n");
		n720_rcv_package_status = rcv_cplt;
		n720_clear_rx();
		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_RESET);
		return 0;
	}

	//ищем начало строки
	while(!((n720_rxBuff[start_pos] == '$')&&
					(n720_rxBuff[start_pos + 1] == 'G')&&
					(n720_rxBuff[start_pos + 2] == 'P'))&&
					(start_pos < 25))start_pos++;
	if(start_pos > 20)//первые 4 символа(NMEA), пятый должен быть($) если пакет смещен в буфере отказваемся от него
	{
		//n720_rcv_package_status = rcv_cplt;
		//n720_clear_rx();
		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_RESET);
		return 0;
	}
	if(checkSum(start_pos) == 0)//если контрольная сумма не совпадает отказываемя от пакета
	{
		//n720_rcv_package_status = rcv_cplt;
		//n720_clear_rx();
		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_RESET);
		return 0;
	}
	
	memset(&gpgga_struct, 0x00, sizeof(gprmc_struct));//зачищаем старое значение
	for(i = start_pos + 6; i < BUFF_LENGTH; i++)//GPGSA + 6 переходим в тело сообщения
	{
		if(n720_rxBuff[i] == ',')
		{
			field_pos++;
			field_symbol_pos = 0;
		}
		else
		switch(field_pos)
		{
			case(1): if(field_symbol_pos <= sizeof(gprmc_struct.utc_time)-1) gprmc_struct.utc_time[field_symbol_pos++] = n720_rxBuff[i]; break;
			case(2): if(field_symbol_pos <= sizeof(gprmc_struct.status_of_position_fix)) gprmc_struct.status_of_position_fix = n720_rxBuff[i]; break;
			case(3): if(field_symbol_pos <= sizeof(gprmc_struct.latitude)-1) gprmc_struct.latitude[field_symbol_pos++]= n720_rxBuff[i]; break;
			case(4): if(field_symbol_pos <= sizeof(gprmc_struct.ns)) gprmc_struct.ns = n720_rxBuff[i]; break;
			case(5): if(field_symbol_pos <= sizeof(gprmc_struct.longitude)-1) gprmc_struct.longitude[field_symbol_pos++] = n720_rxBuff[i]; break;
			case(6): if(field_symbol_pos <= sizeof(gprmc_struct.ew)) gprmc_struct.ew = n720_rxBuff[i]; break;
			case(7): if(field_symbol_pos <= sizeof(gprmc_struct.speed_over_ground_knots)-1) gprmc_struct.speed_over_ground_knots[field_symbol_pos++] = n720_rxBuff[i]; break;
			case(8): if(field_symbol_pos <= sizeof(gprmc_struct.track_made_good)-1)gprmc_struct.track_made_good[field_symbol_pos++] = n720_rxBuff[i]; break;
			case(9): if(field_symbol_pos <= sizeof(gprmc_struct.utc_date)-1) gprmc_struct.utc_date[field_symbol_pos++] = n720_rxBuff[i]; break;
			case(10): if(field_symbol_pos <= sizeof(gprmc_struct.magnetic_variation_degrees)-1) gprmc_struct.magnetic_variation_degrees[field_symbol_pos++] = n720_rxBuff[i]; break;
			case(11): if(field_symbol_pos <= sizeof(gprmc_struct.mvd_ew)) gprmc_struct.mvd_ew = n720_rxBuff[i]; break;
			case(12): if(field_symbol_pos <= sizeof(gprmc_struct.gps_quality_indicator))gprmc_struct.gps_quality_indicator = n720_rxBuff[i]; break;
			//case(17): return 1;
			default: break;
		}
	}
	if(gprmc_struct.status_of_position_fix == 'V')//если геопозиция invalid
	{
		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_RESET);//гасим светодиод GPS
		//n720_clear_rx();
		memset(&gprmc_struct, 0x00, sizeof(gprmc_struct));
		//n720_rcv_package_status = rcv_cplt;
		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_RESET);
		return 0;
	}else{
		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_SET);//зажигаем светодиод GPS
	}
	set_float_gprmc();
	if(knots_to_kmh() == 0.0)return 2;//если скорость нулевая пробег не считаем, но координаты есть
	//n720_clear_rx();
	HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_SET);//даем индикацию получения данных GPS
	return 1;
}
//---------------------------------GPS_parser-------------------------
uint8_t gps_parser(void)
{
	uint8_t field_pos = 0;//счетчик полей сообщения по разделителям ','
	uint8_t field_symbol_pos = 0;//счетчик символов в каждом поле струтуры
	uint16_t start_pos = 0, i = 0;
	
	//ищем начало строки
	while(!((n720_rxBuff[start_pos] == '$')&&
					(n720_rxBuff[start_pos + 1] == 'P')&&
					(n720_rxBuff[start_pos + 2] == 'N')&&
					(n720_rxBuff[start_pos + 3] == 'G')&&
					(n720_rxBuff[start_pos + 4] == 'P')&&
					(n720_rxBuff[start_pos + 5] == 'S'))&&
					(start_pos < 25))start_pos++;
	if(start_pos > 20)//первые 4 символа(NMEA), пятый должен быть($) если пакет смещен в буфере отказваемся от него
	{
		n720_rcv_package_status = rcv_cplt;
		n720_clear_rx();
		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_RESET);
		return 0;
	}
	if(checkSum(start_pos) == 0)//если контрольная сумма не совпадает отказываемя от пакета
	{
		n720_rcv_package_status = rcv_cplt;
		n720_clear_rx();
		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_RESET);
		return 0;
	}
	
	memset(&gps_struct, 0x00, sizeof(gps_struct));//зачищаем старое значение
	for(i = start_pos + 6; i < BUFF_LENGTH; i++)//PNGPS + 6 переходим в тело сообщения
	{
		if(n720_rxBuff[i] == ',')
		{
			field_pos++;
			field_symbol_pos = 0;
		}
		else
		switch(field_pos)
		{
			case(1): if(field_symbol_pos <= sizeof(gps_struct.sn)-1) gps_struct.sn[field_symbol_pos++] = n720_rxBuff[i]; break;
			case(2): if(field_symbol_pos <= sizeof(gps_struct.time_stamp)-1) gps_struct.time_stamp[field_symbol_pos++] = n720_rxBuff[i]; break;
			case(3): if(field_symbol_pos <= sizeof(gps_struct.latitude)-1) gps_struct.latitude[field_symbol_pos++]= n720_rxBuff[i]; break;
			case(4): if(field_symbol_pos <= sizeof(gps_struct.latitude)-1) gps_struct.latitude[field_symbol_pos++] = n720_rxBuff[i]; break;
			case(5): if(field_symbol_pos <= sizeof(gps_struct.status)) gps_struct.status = n720_rxBuff[i]; break;
			default: break;
		}
	}
	HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_SET);//зажигаем светодиод GPS
	if(gps_struct.status != 'Y')//проверяем статус пакета
	{
		n720_clear_rx(); 
		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_RESET);//данные GPS некорректны
		return 0;//если нет движения или обрыв антенны
	}
	//переводим unix time в нормальный вид
	uint32_t unix_time = atoi(gps_struct.time_stamp);
	
	char *ptr = convert_unix_time(unix_time);//функция возвращает укаатель на массив, с помошью которого мы копируем данные в нашу структуру
	for(uint8_t i = 0; i < 9; i++)//размер массива 9 символов
	{
		gps_struct.time[i] = *ptr;
		ptr++;
	}
	ptr = convert_unix_date(unix_time);
	for(uint8_t i = 0; i < 6; i++)//размер массива 6 символов
	{
		gps_struct.date[i] = *ptr;
		ptr++;
	}
	HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_SET);//индикация наличия сигнала GPS
	return 1;
}

//--------------принять ОК-------------------
uint8_t get_ok(void)
{
	uint16_t start_pos = 0;
	if(n720_ack_timeout(5000) == 0)
	{
		n720_rcv_package_status = rcv_cplt;
		n720_clear_rx();
		n720_uart_rcv_package_it();
		return 0;//ждем приема
	}
	while(!((n720_rxBuff[start_pos] == '\r')&&
					(n720_rxBuff[start_pos + 1] == '\n')&&
					(n720_rxBuff[start_pos + 2] == 'O')&&
					(n720_rxBuff[start_pos + 3] == 'K')&&
					(n720_rxBuff[start_pos + 4] == '\r')&&
					(n720_rxBuff[start_pos + 5] == '\n'))&&
					(start_pos < 25))start_pos++;
	if(start_pos > 20)
	{
		n720_rcv_package_status = rcv_cplt;
		n720_clear_rx();
		n720_uart_rcv_package_it();
		return 0;
	}
	n720_clear_rx();
	n720_rcv_package_status = rcv_cplt;
	n720_uart_rcv_package_it();
	return 1;
}

//первоначальная настройка модема с помошью АТ команд
void at_init(void)
{
	uint8_t i;
	for(i = 0; i < 3; i++)
	{

		sprintf((char*)n720_txBuff, "AT\r\n");//проверяем связь
		n720_uart_transmit_package_dma();
		if(get_ok() == 0)continue;

		sprintf((char*)n720_txBuff, "ATE0\r\n");//отключаем эхо
		n720_uart_transmit_package_dma();
		//if(n720_ack_timeout(5000) == 0)continue;
		if(get_ok() == 0)continue;
		
		sprintf((char*)n720_txBuff, "AT$MYGPSPWR=1\r\n");//даем питание на gps
		n720_uart_transmit_package_dma();	
		if(get_ok() == 0)continue;
		break;
	}
	if(i == 3)
	{
		sprintf((char*)service_txBuff, "AT COMMANDS: INITIALIZATION ERROR\r\nNEOWAY USART2 NOT RESPONDED\r\nREBOOT...\r\n");
		service_uart_transmit_package_dma();
		//перезагружаемся
		HAL_NVIC_SystemReset();
	}
}
//-----------------------------ответ на запрос пробега---------------
	/*
		предполагаю, что структура пакета будет такова:
		Запрос к STM: #ATMIL,<байт сброса промежуточного пробега><CR><LF>
												<байт сброса промежуточного пробега>:
													0 - не трогаем промежуточный пробег
													1 - сбрасываем промежуточный пробег

	Ответ STM: #ATMIL,<полный пробег вагона>,<промежуточный пробег вагона>*<CRC8><CR><LF>
												<CRC8> - использую потому, что стандарт NMEA 
																 предполагает исполльзование именно CRC8
																 можно заменить на CRC32
	*/

uint8_t send_mileage(void)
{
	uint16_t start_pos = 0;

		while(!((n720_rxBuff[start_pos] == '#')&&
					(n720_rxBuff[start_pos + 1] == 'A')&&
					(n720_rxBuff[start_pos + 2] == 'T')&&
					(n720_rxBuff[start_pos + 3] == 'M')&&
					(n720_rxBuff[start_pos + 4] == 'I')&&
					(n720_rxBuff[start_pos + 5] == 'L'))&&
					(start_pos < 25))start_pos++;
		if(start_pos > 20)//если сообщение в буфере смещено
		{
			sprintf((char*)n720_txBuff, "ERROR");
			n720_uart_transmit_package_dma();
			n720_rcv_package_status = rcv_cplt;
			n720_clear_rx();
			return 0;
		}
		uint32_t total_mil = get_temp_total_mil();//запрашиваем пробег
		uint32_t subtotal_mil = get_temp_subtotal_mil();
		uint8_t crc;
		sprintf((char*)n720_txBuff, "ATMIL,%d,%d*", total_mil, subtotal_mil);//готовимся к подсчету контрольной суммы
		for(uint16_t i = 0; i < (strlen((char*)n720_txBuff) - 1); i++)
		{
			crc ^= n720_txBuff[i];//считаем контрольную сумму
			n720_txBuff[i] = 0x00;//зачищаем массив
		}
		sprintf((char*)n720_txBuff, "ATMIL,%d,%d*%d", total_mil, subtotal_mil, crc);//формируем сообщение
		n720_uart_transmit_package_dma();//отправляем
		if(n720_rxBuff[start_pos + 7] == '1')//если получили команду на сброс промежуточного пробега
		{
			reset_temp_subtotal_mil();
		}
		n720_rcv_package_status = rcv_cplt;
		return 1;
}
		/*
		switch(n720_rxBuff[start_pos + 7])//+ 7 смотрим символ сброса промежуточного побега 1 - сбрасываем, 0 - оставляем как было
		{
			case('0'):
			{
				uint32_t total_mil = get_temp_total_mil();
				uint32_t subtotal_mil = get_temp_subtotal_mil();
				uint8_t crc;
				sprintf((char*)n720_txBuff, "ATMIL,%d,%d,0*", total_mil, subtotal_mil);
				for(uint16_t i = 0; i < (strlen((char*)n720_txBuff) - 1); i++)
				{
					crc ^= n720_txBuff[i];//считаем контрольную сумму
					n720_txBuff[i] = 0x00;//зачищаем массив
				}
				sprintf((char*)n720_txBuff, "ATMIL,%d,%d,0*%d", total_mil, subtotal_mil, crc);
				n720_uart_transmit_package_dma();
				n720_rcv_package_status = rcv_cplt;
				return 1;
			}
			case('1'):
			{
				uint32_t total_mil = get_temp_total_mil();
				uint32_t subtotal_mil = get_temp_subtotal_mil();
				uint8_t crc;
				sprintf((char*)n720_txBuff, );
			}
		}
		*/


/*
									РЕЗЕРВ
								GGA_PARSER


//добавляем точку после градусов в строчный массив, и убираем точку после минут
//void set_float(void)
//{
//	uint8_t arr_size;
//	arr_size = (sizeof(gpgga_struct.latitude)-1);
//	uint8_t i = arr_size;
//	for(; i >= 2; i--)
//	{
//		if(gpgga_struct.latitude[i] == 0x00)continue;
//		gpgga_struct.latitude[i + 1] = gpgga_struct.latitude[i];
//		if(i == 2)gpgga_struct.latitude[i] = '.';
//	}
//	//ищем вторую точку, которую требуется очистить 
//	i = arr_size;
//	for(; i >= 2; i--)
//	{
//		if(gpgga_struct.latitude[i] == 0x00)continue;
//		if(gpgga_struct.latitude[i] == '.')break;
//	}
//	for(; i < arr_size; i++)
//	{
//		if(i == 2)break;//если в предыдущем цикле нашли точку после градусов - выходим
//		gpgga_struct.latitude[i] = gpgga_struct.latitude[i + 1];
//		if(gpgga_struct.latitude[i] == 0x00)break;//выход из пустого массива
//	}
//	//для второго массива
//	arr_size = (sizeof(gpgga_struct.longitude)-1);
//	i = arr_size;
//	for(; i >= 3; i--)
//	{
//		if(gpgga_struct.longitude[i] == 0x00)continue;
//		else gpgga_struct.longitude[i + 1] = gpgga_struct.longitude[i];
//		if(i == 3)gpgga_struct.longitude[i] = '.';
//	}
//	//ищем вторую точку, которую требуется очистить 
//	i = arr_size;
//	for(; i >= 3; i--)
//	{
//		if(gpgga_struct.longitude[i] == 0x00)continue;
//		if(gpgga_struct.longitude[i] == '.')break;
//	}
//	for(; i < arr_size; i++)
//	{
//		if(i == 3)break;
//		gpgga_struct.longitude[i] = gpgga_struct.longitude[i + 1];
//		if(gpgga_struct.longitude[i] == 0x00)break;
//	}
//}


//запрашиваем координату
//uint8_t GGA_parser(void)
//{	
//	uint8_t field_pos = 0;//счетчик полей сообщения по разделителям ','
//	uint8_t field_symbol_pos = 0;//счетчик символов в каждом поле струтуры
//	uint16_t start_pos = 0, i = 0;
//	
//	//запрашиваем пакет
//	sprintf((char*)n720_txBuff, "AT$MYGPSPOS=0\r");//отправляем запрос
//	n720_uart_transmit_package_dma();
//	if(n720_ack_timeout(2000) == 0)//таймаут ответа
//	{
//		sprintf((char*)n720_txBuff, "TIMEOUT\r");
//		n720_uart_transmit_package_dma();
//		n720_rcv_package_status = rcv_cplt;
//		n720_clear_rx();
//		return 0;
//	}

//	//ищем начало строки
//	while(!((n720_rxBuff[start_pos] == '$')&&
//					(n720_rxBuff[start_pos + 1] == 'G')&&
//					(n720_rxBuff[start_pos + 2] == 'P'))&&
//					(start_pos < 25))start_pos++;
//	if(start_pos > 20)//первые 4 символа(NMEA), пятый должен быть($) если пакет смещен в буфере отказваемся от него
//	{
//		n720_rcv_package_status = rcv_cplt;
//		n720_clear_rx();
//		return 0;
//	}
//	if(checkSum(start_pos) == 0)//если контрольная сумма не совпадает отказываемя от пакета
//	{
//		n720_rcv_package_status = rcv_cplt;
//		n720_clear_rx();
//		return 0;
//	}
//	memset(&gpgga_struct, 0x00, sizeof(gpgga_struct));//зачищаем старое значение
//	for(i = start_pos + 6; i < BUFF_LENGTH; i++)//GPGSA
//	{
//		if(n720_rxBuff[i] == ',')
//		{
//			field_pos++;
//			field_symbol_pos = 0;
//		}
//		else
//		switch(field_pos)
//		{
//			case(1): if(field_symbol_pos <= sizeof(gpgga_struct.utc_time)-1) gpgga_struct.utc_time[field_symbol_pos++] = n720_rxBuff[i]; break;
//			case(2): if(field_symbol_pos <= sizeof(gpgga_struct.latitude)-1) gpgga_struct.latitude[field_symbol_pos++] = n720_rxBuff[i]; break;
//			case(3): if(field_symbol_pos <= sizeof(gpgga_struct.ns)) gpgga_struct.ns = n720_rxBuff[i]; break;
//			case(4): if(field_symbol_pos <= sizeof(gpgga_struct.longitude)-1) gpgga_struct.longitude[field_symbol_pos++] = n720_rxBuff[i]; break;
//			case(5): if(field_symbol_pos <= sizeof(gpgga_struct.we)) gpgga_struct.we = n720_rxBuff[i]; break;
//			case(6): if(field_symbol_pos <= sizeof(gpgga_struct.GPS_quality)) gpgga_struct.GPS_quality = n720_rxBuff[i]; break;
//			case(7): if(field_symbol_pos <= sizeof(gpgga_struct.satellites_in_use)-1) gpgga_struct.satellites_in_use[field_symbol_pos++] = n720_rxBuff[i]; break;
//			//case(17): return 1;
//			default: break;
//		}
//	}
//	if(gpgga_struct.GPS_quality == '0')//если обрыв антенны или нет GPS_fix
//	{
//		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_RESET);//гасим светодиод GPS
//		memset(&gpgga_struct, 0x00, sizeof(gpgga_struct));
//		//n720_clear_rx();
//		n720_rcv_package_status = rcv_cplt;
//		return 0;
//	}else{
//		HAL_GPIO_WritePin(GPS_LED_GPIO_Port, GPS_LED_Pin, GPIO_PIN_SET);//зажигаем светодиод GPS
//	}
//	set_float();
//	return 1;
//}


									
									3D-fix parser

//смотрим 3D-fix
//получаем информацию о 3d fix
uint8_t gps_fix_parser(void)
{
	uint8_t field_pos = 0;//счетчик полей сообщения по разделителям ','
	uint16_t start_pos = 0, i = 0;
	//запрашиваем пакет
	sprintf((char*)n720_txBuff, "AT$MYGPSPOS=NMEA$GPGSA\r");
	n720_uart_transmit_package_dma();

	//ищем начало строки
	while(!((n720_rxBuff[start_pos] == '$')&&
					(n720_rxBuff[start_pos + 1] == 'G')&&
					(n720_rxBuff[start_pos + 2] == 'P'))&&
					(start_pos < 8))start_pos++;
	if(start_pos > 5)return 0;//первые 4 символа(NMEA), пятый должен быть($) если пакет смещен в буфере отказваемся от него
	if(checkSum(start_pos) == 0)return 0;//если контрольная сумма не совпадает отказываемя от пакета
	memset(&fix_type, 0x00, sizeof(fix_type));//зачищаем старое значение
	for(i = start_pos + 6; i < BUFF_LENGTH; i++)//GPGSA
	{
		if(n720_rxBuff[i] == ',')
		{
			field_pos++;
		}
		else
		switch(field_pos)
		{
			case(2): fix_type = n720_rxBuff[i];
			case(17): return 1;
			default: break;
		}
	}
	return 1;
}



*/
/*
									РЕЗЕРВ
	АТ команда, позволяющая завершить все процессы 
	и сохранить данные на neoway	
*/
uint8_t n720_sleep(void)
{
	uint32_t temp_total_mil = get_temp_total_mil();
	uint32_t temp_subtotal_mil = get_temp_subtotal_mil();
	
	uint8_t i = 0;
	for(i = 0; i < 3; i++)
	{
		sprintf((char*)n720_txBuff, "SLEEP\r\n");
		n720_uart_transmit_package_dma();
		if(get_ok() == 0)continue;
		break;
	}
	//снимаем питание с neoway
	HAL_GPIO_WritePin(MODEM_OFF_GPIO_Port, MODEM_OFF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(N720_POWER_GPIO_Port, N720_POWER_Pin, GPIO_PIN_RESET);
	if(i == 3)
	{
		sprintf((char*)service_txBuff, "AT COMMANDS: SLEEP_CMD\r\nNEOWAY USART2 NOT RESPONDED\r\nFORCED REBOOT\r\n");
		service_uart_transmit_package_dma();
		return 0;
	}
	return 1;
}

