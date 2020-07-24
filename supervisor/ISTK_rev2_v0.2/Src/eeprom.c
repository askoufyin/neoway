#include "eeprom.h"
#include "uart.h"
#include "nmea_parser.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;
//extern TIM_HandleTypeDef htim6;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

//структура, из которой берем дату и время
extern gprmc_struct_type_def gprmc_struct;

extern uint8_t n720_rxBuff[BUFF_LENGTH];
extern uint8_t n720_txBuff[BUFF_LENGTH];

extern uint8_t service_rxBuff[BUFF_LENGTH];
extern uint8_t service_txBuff[BUFF_LENGTH];

extern volatile uint8_t service_mode;//при 1 в сервисный UART отправляется отладочная информация

memory block1;
const memory mark;
memory *p_block1 = &block1;
uint8_t eeprom_error_flag = 0;

//максимальное кол-во записей на ячейку 4000000
//219000 записей в год при записи раз в 10 минут
void eeprom_error_handler(void)
{
	service_uart_transmit_str_dma("EEPROM_ERROR\r\n");
	eeprom_error_flag = 1;
//	HAL_TIM_Base_Start_IT(&htim6);
}

//-----------------WRITE ENABLE(WREN)-выполняется перед каждой командой WRITE и WRSR---------------
uint8_t eeprom_wren(void)
{
	uint8_t cmd_wren = EEPROM_WREN;
	HAL_StatusTypeDef status;
	EEPROM_CS_LOW();
	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);//разрешаем запись
	status = HAL_SPI_Transmit_DMA(&hspi1, &cmd_wren, 1);
	while(hspi1.hdmatx->State != HAL_DMA_STATE_READY);//ждем пока DMA отправит данные
	EEPROM_CS_HIGH();
	if(status == HAL_ERROR)eeprom_error_handler();
	return 1;
}

//-------------------читаем регистр статуса(RDSR)-------------------------------------------------
uint8_t eeprom_rdsr(void)
{
	uint8_t cmd_rdsr = EEPROM_RDSR;
	uint8_t res_rdsr[2] = {0,};
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);//разрешаем запись
	EEPROM_CS_LOW();
	status = HAL_SPI_TransmitReceive_DMA(&hspi1, &cmd_rdsr, res_rdsr, 2);
	while(hspi1.hdmarx->State != HAL_DMA_STATE_READY);
	EEPROM_CS_HIGH();
	if(status == HAL_ERROR)eeprom_error_handler();
	return res_rdsr[1];
}

//----------------READ FROM MEMORY ARRAY (READ)-чтение из памяти-------------------------------------
uint8_t eeprom_read_byte(uint16_t addr)
{
//	HAL_UART_AbortReceive_IT(&huart2);//останавливаем прием данных от neoway
//	HAL_UART_AbortReceive_IT(&huart1);
	
	uint8_t cmd_read_byte[3] = {0,};
	cmd_read_byte[0] = EEPROM_READ;//команда "чтение"
	cmd_read_byte[1] = addr >> 8;//старший байт адреса чтения
	cmd_read_byte[2] = addr;//младший байт адреса чтения
	uint8_t res_read[4] = {0,};//ответ 
	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);//разрешаем запись
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);//разрешаем запись
	EEPROM_CS_LOW();
	status = HAL_SPI_TransmitReceive_DMA(&hspi1, cmd_read_byte, res_read, 4);
	while(hspi1.hdmarx->State != HAL_DMA_STATE_READY);
	EEPROM_CS_HIGH();
	if(status == HAL_ERROR)eeprom_error_handler();
	
//	n720_uart_rcv_package_it();//после записи возобновляем прием данных
//	service_uart_rcv_package_it();
	
	return res_read[3];
}

//---------------WRITE TO MEMORY ARRAY (WRITE)-запись в память--------------------------------------
uint8_t eeprom_write_byte(uint16_t addr, uint8_t data)
{
//	HAL_UART_AbortReceive_IT(&huart2);//останавливаем прием данных от neoway
//	HAL_UART_AbortReceive_IT(&huart1);
	uint8_t cmd_write_byte[4] = {0,};
	cmd_write_byte[0] = EEPROM_WRITE;//команда записи
	cmd_write_byte[1] = addr >> 8;//старшие 8 бит адреса записи
	cmd_write_byte[2] = addr;//младшие 8 бит адреса записи
	cmd_write_byte[3] = data;//записываемые данные
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(HOLD_GPIO_Port, HOLD_Pin, GPIO_PIN_SET);//разрешаем запись
	//в цикле записываем и вычитывем записанные данныне,
//	if(eeprom_read_byte(addr) != data)while(1);
	//если из 3 раз не получится вычитать записываемые данные
	//уходим в ошибку
	for(uint8_t i = 0; i < 3; i++)
	{
		eeprom_wren();//отправляем команду "разрешить запись"
		EEPROM_CS_LOW();
		status = HAL_SPI_Transmit_DMA(&hspi1, cmd_write_byte, 4);
		while(hspi1.hdmatx->State != HAL_DMA_STATE_READY);
		EEPROM_CS_HIGH();
		if(status == HAL_ERROR)eeprom_error_handler();
		while(eeprom_rdsr() != 0x00);//ждем пока произойдет запись 5ms (регистр статуса должен быть 0х00)
//		n720_uart_rcv_package_it();//после записи возобновляем прием данных
//		service_uart_rcv_package_it();
		return 1;
//		if(eeprom_read_byte(addr) == data)return 1;
	}
	return 0;
}
//-----------------------read_all----------------
void eeprom_read_all(void)
{
	for(uint16_t i = 0; i < EEPROM_SIZE; i++)
	{
		sprintf((char*)service_txBuff, "%d    %0x\r\n", i, eeprom_read_byte(i));
		service_uart_transmit_package_dma();
	}
}
//-----------------------erase all data-----------------------
void eeprom_erase(void)
{
	for(uint16_t i = 0; i < EEPROM_SIZE; i++)
	{
		if(i % 1000 == 0)service_uart_transmit_str_dma(".");
		if(eeprom_write_byte(i, 0xFF) == 0)eeprom_error_handler();
	}
	service_uart_transmit_str_dma("\r\n");
}

//----------------------check_crc-----------------------------
uint8_t eeprom_create_checksum(void)
{
	uint8_t crc = 0;
	for(uint8_t i = 0; i < TOTAL_MILEAGE_BYTES; i++)crc ^= block1.total_mileage[i];
	for(uint8_t i = 0; i < SUBTOTAL_MILEAGE_BYTES; i++)crc ^= block1.subtotal_mileage[i];
	for(uint8_t i = 0; i < TIME_BYTES; i++)crc ^= block1.time[i];
	for(uint8_t i = 0; i < DATE_BYTES; i++)crc ^= block1.date[i];
	block1.checksum = crc;
	return crc;
}


uint8_t eeprom_check_crc(void)
{
	uint8_t crc = 0;
	for(uint8_t i = 0; i < TOTAL_MILEAGE_BYTES; i++)crc ^= block1.total_mileage[i];
	for(uint8_t i = 0; i < SUBTOTAL_MILEAGE_BYTES; i++)crc ^= block1.subtotal_mileage[i];
	for(uint8_t i = 0; i < TIME_BYTES; i++)crc ^= block1.time[i];
	for(uint8_t i = 0; i < DATE_BYTES; i++)crc ^= block1.date[i];
	
	if(crc == block1.checksum)return 1;
//	else if(crc != block1.checksum)return 0;
	return 0;
}

//----------------------read block of data----------------------
uint8_t eeprom_read_block(uint8_t block_number)
{
	HAL_UART_AbortReceive_IT(&huart2);//останавливаем прием данных от neoway
	HAL_UART_AbortReceive_IT(&huart1);
	if(block_number > ((EEPROM_SIZE / EEPROM_BLOCK_SIZE) - 1))block_number = 0;
	block1.cell_num = block_number;
	uint16_t addr = block_number * EEPROM_BLOCK_SIZE;
	//читаем пробег
	for(uint8_t i = 0; i < TOTAL_MILEAGE_BYTES; i++)
	{
		block1.total_mileage[i] = eeprom_read_byte(addr++);
	}
	//промежуточный пробег
	for(uint8_t i = 0; i < SUBTOTAL_MILEAGE_BYTES; i++)
	{
		block1.subtotal_mileage[i] = eeprom_read_byte(addr++);
	}
	//время
	for(uint8_t i = 0; i < TIME_BYTES; i++)
	{
		block1.time[i] = eeprom_read_byte(addr++);
	}
	for(uint8_t i = 0; i < DATE_BYTES; i++)
	{
		block1.date[i] = eeprom_read_byte(addr++);
	}
	block1.checksum = eeprom_read_byte(addr++);
	n720_uart_rcv_package_it();//после записи возобновляем прием данных
	service_uart_rcv_package_it();
	return 1;
}
//-------------------write block of data------------------------
uint8_t eeprom_write_block(uint8_t block_number)
{
	HAL_UART_AbortReceive_IT(&huart2);//останавливаем прием данных от neoway
	HAL_UART_AbortReceive_IT(&huart1);
	if(block_number >= 127)//((EEPROM_SIZE / EEPROM_BLOCK_SIZE)))
	{
		block_number = 0;
	}
	block1.cell_num = block_number;
	uint16_t addr = block_number * EEPROM_BLOCK_SIZE;
	//пишем пробег
	for(uint8_t i = 0; i < TOTAL_MILEAGE_BYTES; i++)
	{
		if(eeprom_write_byte(addr++, block1.total_mileage[i]) == 0)return 0;
	}
	//пишем промежуточный пробег
	for(uint8_t i = 0; i < SUBTOTAL_MILEAGE_BYTES; i++)
	{
		if(eeprom_write_byte(addr++, block1.subtotal_mileage[i]) == 0)return 0;
	}
	//пишем время
	for(uint8_t i = 0; i < TIME_BYTES; i++)
	{
		if((eeprom_write_byte(addr++, block1.time[i]) == 0))return 0;
	}
	//пишем дату
	for(uint8_t i = 0; i < DATE_BYTES; i++)
	{
		if(eeprom_write_byte(addr++, block1.date[i]) == 0)return 0;
	}
	//пишем контрольную сумму
	if(eeprom_write_byte(addr++, block1.checksum) == 0)return 0;
	
	addr += (RESERVED_BYTES+1);//компенсация на зарезервированные ячейки
	//ставим метку
	if(block_number == ((EEPROM_SIZE / EEPROM_BLOCK_SIZE) - 1))addr = 0;
	for(uint8_t i = 0; i < (EEPROM_BLOCK_SIZE); i++)
	{
		if(eeprom_write_byte(addr++, 0xFF) == 0)return 0;
	}
	n720_uart_rcv_package_it();//после записи возобновляем прием данных
	service_uart_rcv_package_it();
	return 1;
}

//----------------------find mark-----------------
// -1 к возврату при чтении
uint8_t eeprom_find_mark(void)
{
	uint8_t sym_cnt = 0;//счетчик одинаковых символов
	for(uint16_t addr = 0; addr < EEPROM_SIZE; addr++)
	{
		//addr -=1;
		while((addr % EEPROM_BLOCK_SIZE) != 0)addr++;
		for(uint8_t i = 0; i < EEPROM_BLOCK_SIZE; i++)
		{
			if(eeprom_read_byte(addr++) != 0xFF)break;
			else sym_cnt++;
		}
		if(sym_cnt == EEPROM_BLOCK_SIZE)
		{
//------В случае сервисного обслуживания--
			if(service_mode == 1)
			{
				sprintf((char*)service_txBuff, "MARK: %d\r\n", ((addr / EEPROM_BLOCK_SIZE) - 1));
				service_uart_transmit_package_dma();
			}
			return ((addr / EEPROM_BLOCK_SIZE) - 1);
		}
	}
	if(service_mode == 1)service_uart_transmit_str_dma("MARK: 0\r\n");
	return 0;
}

//uint8_t eeprom_find_space(void)
//{
//	uint8_t sym_cnt = 0;//счетчик одинаковых символов
//	uint8_t cell_cnt = 0;//счетчик ячеек
//	uint8_t addr = 0;//адрес, по которому осуществляется чтение
//	
//	for(cell_cnt = 0; cell_cnt < 128; cell_cnt++)
//	{
//		sym_cnt = 0;
//		for(uint8_t i = 1; i <= 64; i++)
//		{
//			addr = cell_cnt * i;
//			if(eeprom_read_byte(addr) != 0xFF)break;
//			sym_cnt++;
//			if(sym_cnt == 64) return cell_cnt;
//		}
//	}
//	return 0;
//}

//------------------check eeprom------------------
uint8_t is_eeprom_empty(void)
{
	uint16_t counter;
	for(counter = 0; counter < EEPROM_SIZE; counter++)
	{
		if(eeprom_read_byte(counter) != 0xFF)break;
	}
	if(counter == (EEPROM_SIZE))return 1;
	else return 0;
}
//------------------init--------------------------
/*
	Проверяем eeprom на наличие записанных блоков
	Возвращаем общий пробег
*/
void eeprom_init(void)
{
	//проверяем, пуста ли флешка
	if(is_eeprom_empty() == 1)
	{
		//если пуста, перезаписываем нулями и создаем первую запись
		if(service_mode == 1)service_uart_transmit_str_dma("EPROM IS EMPTY... INIT\r\n");
		for(uint16_t i = 0; i < EEPROM_BLOCK_SIZE; i++)eeprom_write_byte(i, 0x00);
		for(uint8_t i = 0; i < TOTAL_MILEAGE_BYTES; i++)block1.total_mileage[i] = 0x00;
		for(uint8_t i = 0; i < SUBTOTAL_MILEAGE_BYTES; i++)block1.subtotal_mileage[i] = 0x00;
		for(uint8_t i = 0; i < TIME_BYTES; i++)block1.time[i] = 0x00;
		for(uint8_t i = 0; i < DATE_BYTES; i++)block1.date[i] = 0x00;
		block1.checksum = 0;
		eeprom_write_block(0);
		service_uart_transmit_str_dma("EEPROM INIT CPLT\r\n");
	}
}
//----------------erase--------------------
//--------------read write mileage----------------
//---read---
void eeprom_send_block_to_uart(void)
{
	uint8_t temp_addr;
	temp_addr = eeprom_find_mark() - 1;
	eeprom_read_block(temp_addr);
	if(eeprom_check_crc() == 1)service_uart_transmit_str_dma("CRC valid\r\n");
	else service_uart_transmit_str_dma("CRC invalid\r\n");
	sprintf((char*)service_txBuff, "total_mil: %s, subtotal_mil: %s, date: %s, time: %s, crc: %0x\r\n", block1.total_mileage, block1.subtotal_mileage, block1.date, block1.time, block1.checksum);
	service_uart_transmit_package_dma();
}

uint32_t eeprom_read_totalmil(void)
{
	uint32_t total_mil = 0;
	eeprom_read_block(eeprom_find_mark() - 1);
	if(eeprom_check_crc() == 1)service_uart_transmit_str_dma("CRC valid\r\n");
	else service_uart_transmit_str_dma("CRC invalid\r\n");
	total_mil = atoi((char*)block1.total_mileage);
	return total_mil;
}

uint32_t eeprom_read_subtotalmil(void)
{
	uint32_t subtotal_mil = 0;
	eeprom_read_block(eeprom_find_mark() - 1);
	if(eeprom_check_crc() == 1)service_uart_transmit_str_dma("CRC valid\r\n");
	else service_uart_transmit_str_dma("CRC invalid\r\n");
	subtotal_mil = atoi((char*)block1.subtotal_mileage);
	return subtotal_mil;
}

//---write---
//booth mileage
void eeprom_write_mileage(uint32_t total_mil, uint32_t subtotal_mil)
{
	eeprom_read_block(eeprom_find_mark() - 1);//вычитываем первоначальное значение
	sprintf((char*)block1.total_mileage, "%d", total_mil);
	sprintf((char*)block1.subtotal_mileage, "%d", subtotal_mil);
	for(uint8_t i = 0; i < 6; i++)//вносим данные о дате и времени
	{
		block1.time[i] = (uint8_t)gprmc_struct.utc_time[i];
		block1.date[i] = (uint8_t)gprmc_struct.utc_date[i];
	}
	eeprom_create_checksum();//создаем контрольную сумму
	eeprom_write_block(eeprom_find_mark());//записываем данные в EEPROM
}

//total mil
void eeprom_write_totalmil(uint32_t total_mil)
{
	uint8_t temp_addr;
	temp_addr = eeprom_find_mark() - 1;
	eeprom_read_block(temp_addr);//вычитываем первоначальное значение
	sprintf((char*)block1.total_mileage, "%d", total_mil);
	for(uint8_t i = 0; i < 6; i++)//вносим данные о дате и времени
	{
		block1.time[i] = (uint8_t)gprmc_struct.utc_time[i];
		block1.date[i] = (uint8_t)gprmc_struct.utc_date[i];
	}
	eeprom_create_checksum();//создаем контрольную сумму
	temp_addr = eeprom_find_mark();
	eeprom_write_block(temp_addr);//записываем данные в EEPROM
}
//subtotal mil
void eeprom_write_subtotalmil(uint32_t subtotal_mil)
{
	eeprom_read_block(eeprom_find_mark() - 1);//вычитываем первоначальное значение
	sprintf((char*)block1.subtotal_mileage, "%d", subtotal_mil);
	for(uint8_t i = 0; i < 6; i++)//вносим данные о дате и времени
	{
		block1.time[i] = (uint8_t)gprmc_struct.utc_time[i];
		block1.date[i] = (uint8_t)gprmc_struct.utc_date[i];
	}
	eeprom_create_checksum();//создаем контрольную сумму
	eeprom_write_block(eeprom_find_mark());
}
//time
void eeprom_write(char * time)
{
	sprintf((char*)block1.time, "%d", (int)time);
	eeprom_write_block(eeprom_find_mark());
}
//all block
void eeprom_write_data(uint32_t total_mil, uint32_t subtotal_mil, char * date, char * time)
{
	memset(&block1, 0x00, EEPROM_BLOCK_SIZE);//очищаем структуру
	sprintf((char*)block1.total_mileage, "%d", total_mil);
	sprintf((char*)block1.subtotal_mileage, "%d", subtotal_mil);
	strcpy((char*)block1.time, time);
	strcpy((char*)block1.date, date);
	__nop();
	
	eeprom_write_block(eeprom_find_mark());
	//копируем данные из структуры парсера nmea
}

//void eeprom_read_data_row(void)
//{
//	for(uint16_t addr = 0; addr < 2048; addr++)
//	{
//		uint8_t byte;
//		byte = eeprom_read_byte(addr);
//		sprintf((char*)service_txBuff, "ADDR:\t%ul\tDATA\t%0x\r\n", addr, byte);
//		service_uart_transmit_package_dma();
//	}
//}
	//sprintf((char*)block1.unix_time, "%c", (int)*time);
	//eeprom_write_block(eeprom_find_mark());
//}
