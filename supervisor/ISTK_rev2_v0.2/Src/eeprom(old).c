//#include "STM32_EEPROM_SPI.h"

#include "eeprom.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//#include "flags.h"



extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
static int service_mode = 0;
/*
флаг, срабатывает в хендлере при получении прерывания от UART

*/




//#include "stm32f0xx_hal.h"





//-----------------WRITE ENABLE(WREN)-выполняется перед каждой командой WRITE и WRSR---------------
void eepromWREN(void)
{
	uint8_t cmdWREN[1] = {EEPROM_WREN};
	
	EEPROM_CS_LOW();
	HAL_SPI_Transmit(&hspi1, cmdWREN, 1, 0x64);
	EEPROM_CS_HIGH();
}

//-------------------читаем регистр статуса(RDSR)-------------------------------------------------
uint8_t eepromRDSR(void)
{
	uint8_t cmdRSDR[1] = {EEPROM_RDSR};//RDSR command
	uint8_t resRSDR[2];
	
	EEPROM_CS_LOW();//EEPROM_CS_LOW();//set down cs //программный nss, RESET - при установлении соединения, SET при окончании
		
	HAL_SPI_TransmitReceive(&hspi1, cmdRSDR, resRSDR, 2, 0x64);//передаем 1 байт команда, 2 байт принимаем ответ

	EEPROM_CS_HIGH();	
	return resRSDR[1];//первый байт контроллер отправляет команду, а EEPROM молчит, поэтому возвращаем второй байт в котором и будет ответ
}

//----------------READ FROM MEMORY ARRAY (READ)-чтение из памяти-------------------------------------
//uint8_t eepromREAD(uint8_t MSB_ADDR, uint8_t LSB_ADDR)
uint8_t eepromREAD_byte(uint16_t ADDR)
{
	uint8_t cmdRead_byte[3];
	cmdRead_byte[0] = EEPROM_READ;//команда "чтение"
	cmdRead_byte[1] = ADDR >> 8;//старший байт дареса чтения
	cmdRead_byte[2] = ADDR;//младший байт адреса чтения
	uint8_t resRead[4];//ответ
//	HAL_StatusTypeDef status;
	
	EEPROM_CS_LOW();
	
	HAL_SPI_TransmitReceive(&hspi1, cmdRead_byte, resRead, 4, 0x64);

	EEPROM_CS_HIGH();
	return resRead[3];//потому что первые 3 байта EEPROM молчит (см. логический анализатор)
}

//---------------WRITE TO MEMORY ARRAY (WRITE)-запись в память--------------------------------------
void eepromWRITE_byte(uint16_t ADDR, uint8_t DATA)//
{
	uint8_t a = 0;//кол-во попыток записи в ячейку
	do{
	uint8_t cmdWRITE_byte[4];
	cmdWRITE_byte[0] = EEPROM_WRITE;
	cmdWRITE_byte[1] = ADDR >> 8;//первые 8 бит 16 битного адреса
	cmdWRITE_byte[2] = ADDR;//вторые 8 бит 16 битного адреса
	cmdWRITE_byte[3] = DATA;//записываемые данные
	
	eepromWREN();
		
	EEPROM_CS_LOW();
	
	HAL_SPI_Transmit(&hspi1, cmdWRITE_byte, 4, 0x64);
	
	EEPROM_CS_HIGH();
	
	while(eepromRDSR() != 0x00)//ждем пока произойдет запись 5ms (регистр статуса должен быть 0х00)
	{
		
	}
	a++;
	}while(eepromREAD_byte(ADDR) != DATA && a <= 3);//вычитываем записанные данные
	if(eepromREAD_byte(ADDR) != DATA)//если записываемые данные не соответствуют вычитываемым
	{
	eepromError_Handler(ERROR_RW);//уходим в ошибку чтения/записи
	}else{
	return;
	}
}

	
//----------------------------------------------------------------
//	if (eepromREAD(ADDR) == DATA)//вычитываем записанные данные
//	{
//											//если данные читаются
//	}else{							//если данные не читаются
//		uint8_t a = 0;
//		do{								//перезаписываем 2 раза
//			uint8_t cmdWRITE[4];
//			cmdWRITE[0] = EEPROM_WRITE;
//			cmdWRITE[1] = ADDR >> 8;//первые 8 бит 16 битного адреса
//			cmdWRITE[2] = ADDR;//вторые 8 бит 16 битного адреса
//			cmdWRITE[3] = DATA;//записываемые данные
//			EEPROM_CS_LOW();
//			HAL_SPI_Transmit(&hspi1, cmdWRITE, 4, 0x64);
//			EEPROM_CS_HIGH();
//			while(eepromRDSR() != 0x00)//ждем пока произойдет запись (регистр статуса должен быть 0х00)
//			{
//		
//			}
//			a++;
//			}while(a > 3);
//			}
//-----------------------------------------------------------------

//		char arr[32];
//		sprintf(arr, "write in progress... \r\n");
//		HAL_UART_Transmit(&huart1, (uint8_t*)arr, sizeof(arr), 0x64);

uint8_t eepromBUFF[EEPROM_BUFFER_SIZE];


//--------------------------пишем регистр статуса (WRSR)---------------------------------------
void eepromWRSR(uint8_t REG)
{
	eepromWREN();
	uint8_t cmdWRSR[2];
	cmdWRSR[0] = 0x01;
	cmdWRSR[1] = REG;
	EEPROM_CS_LOW();
	HAL_SPI_Transmit(&hspi1, cmdWRSR, 2, 0x64);
	EEPROM_CS_HIGH();
}
	
//--------------------ищем свободное место для записи------------------------------------------	
uint16_t eepromFIND_SPACE_byte(void)
{
uint16_t i;
for(i = 0x0000; i <= EEPROM_SIZE; i++)
	{
		uint8_t b = eepromREAD_byte(i);
		if(eepromREAD_byte(i) == 0x00 && eepromREAD_byte(i + 1) == 0x00 && eepromREAD_byte(i + 2) == 0x00)
		{
			break;
		}
		if(i == EEPROM_SIZE)
		{
			i = 0xFFFF;
		}
	}
	return i;
}
//-----------------------стираем всю память----------------------------------------------------
void eepromERASE(void)
{
//	for(uint16_t i = 0xFF; i <= EEPROM_SIZE; i++)//!!!!!!!!!!!!!!!!!!!
	for(uint16_t i = 0x00; i <= EEPROM_SIZE; i++)
	{
		eepromWREN();
		eepromWRITE_byte(i, 0x00);
	}
}
//---------------------------читаем всю память--------------------------------------------------
void eepromREAD_all(void)
{
uint16_t i;//счетчик адресов 
char arr1[18];
memset(arr1, 0x20, 18);
	
for(i = 0x0000; i < EEPROM_SIZE; i++)
	{ 
		uint8_t b = eepromREAD_byte(i);//данные с адреса
		sprintf(arr1, "%0x\t%0x\r\n", i, b);
		HAL_UART_Transmit(&huart2, (uint8_t*)arr1, 9, 0xFFFF);
		memset(arr1, 0x20, 18);	
		IWDG->KR = 0x0000AAAAU;//сбрасываем вочдог
	}
}

//---------------------EEPROM ERROR HANDLER-------------------------------------
	void eepromError_Handler(error error_type)
	{
		if(error_type == ERROR_RW)
		{
			char arr1[32];
			sprintf(arr1, "ERROR: READ/WRITE\r");
			HAL_UART_Transmit(&huart2, (uint8_t*)arr1, 18, 100);
		}
		if(error_type == ERROR_UART)
		{
			char arr1[32];
			sprintf(arr1, "ERROR: UART\r");
			HAL_UART_Transmit(&huart2, (uint8_t*)arr1, 12, 100);
		}
		if(error_type == ERROR_ACCEL)
		{
			char arr1[32];
			sprintf(arr1, "ERROR: ACCEL\r");
			HAL_UART_Transmit(&huart2, (uint8_t*)arr1, 13, 100);
		}
		if(error_type == ERROR_N720)
		{
			char arr1[32];
			sprintf(arr1, "ERROR: N720\r");
			HAL_UART_Transmit(&huart2, (uint8_t*)arr1, 12, 100);
		}
		
		HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
	}
//-----------------------стркутура ячеек памяти----------------------------------

static memory block;
memory *p_block = &block;


	//---------------------ЧИСТИМ_БУФЕР_EEPROM------------------------------------
void clear_total_mileage(void)
{
	for(uint8_t i = 0; i < (TOTAL_MILEAGE_BYTES); i++)
	{
		block.total_mileage[i] = 0x00;
	}
}
void clear_subtotal_mileage(void)
{
	for(uint8_t i = 0; i < (SUBTOTAL_MILEAGE_BYTES); i++)
	{
		block.subtotal_mileage[i] = 0x00;
	}
}
void clear_time(void)
{
	for(uint8_t i = 0; i < (TIME_BYTES); i++)
	{
		block.unix_time[i] = 0x00;
	}
}

void clear_eepromBUFFER(void)
{
	clear_total_mileage();
	clear_subtotal_mileage();
	clear_time();
}
//--------------------------------------------------------------------------
	

//--------------------чтение------------------------------------------------
void eepromREAD_block (uint16_t ADDR)//чтенние
{
	if(ADDR != 0)
	{
		ADDR = ADDR - (EEPROM_BLOCK_SIZE * 2); //вычитаем метку и читаемые данные
	}
	for(uint8_t i = 0; i < TOTAL_MILEAGE_BYTES; i++)//читаем пробег
	{
		p_block -> total_mileage[i] = eepromREAD_byte(ADDR);
		ADDR++;
	}
	for(uint8_t i = 0; i < SUBTOTAL_MILEAGE_BYTES; i++)//читаем промежуточный пробег
	{
		p_block -> subtotal_mileage[i] = eepromREAD_byte(ADDR);
		ADDR++;
	}
	for(uint8_t i = 0; i < TIME_BYTES; i++)//читаем время
	{
		p_block -> unix_time[i] = eepromREAD_byte(ADDR);
		ADDR++;
	}
}

//---------------------ЗАПИСЬ_БЛОКА_ПАМЯТИ------------------------------------------
void eepromWRITE_block(uint16_t ADDR)//запись
{
	if (ADDR != 0)
	{
		ADDR = ADDR - EEPROM_BLOCK_SIZE; //вычитаем метку
	}
	for(uint8_t i = 0; i < TOTAL_MILEAGE_BYTES; i++)   //пишем пробег
	{
		eepromWRITE_byte(ADDR, p_block -> total_mileage[i]);
		ADDR++;
	}
	for(uint8_t i = 0; i < SUBTOTAL_MILEAGE_BYTES; i++)
	{
		eepromWRITE_byte(ADDR, p_block -> subtotal_mileage[i]); //пишем промежуточный пробег
		ADDR++;
	}
	for(uint8_t i = 0; i < TIME_BYTES; i++) //пишем время
	{
		eepromWRITE_byte(ADDR, p_block -> unix_time[i]);
		ADDR++;
	}
	for(uint8_t i = 0; i < (TOTAL_MILEAGE_BYTES + SUBTOTAL_MILEAGE_BYTES + TIME_BYTES); i++)//ставим метку с которой начнем запись в следующий раз(метка FF 32 раза - записываем всю ячейку в FF)
	{
		eepromWRITE_byte(ADDR, 0xFF);
		ADDR++;
	}
		memset(p_block -> total_mileage, 0x00, TOTAL_MILEAGE_BYTES);
		memset(p_block -> subtotal_mileage, 0x00, SUBTOTAL_MILEAGE_BYTES);
		memset(p_block -> unix_time, 0x00, TIME_BYTES);
}
//------------------------------------------------------------

//-----------------ищем метку для начала записи---------------
uint16_t eepromFIND_SPACE()
{
/*
Метка это 1 блок в котором все байты помечены как 0xFF. ищем метку, последовательно вычитывая 
каждую ячейку. Как только метка найдена, возвращаем адрес начала метки. С него будем производить запись
*/
	uint16_t ADDR = 0;
	uint8_t counter;
	counter = 0;//счетчик одинаковых байт
	
	for(ADDR = 0; ADDR < EEPROM_SIZE; ADDR++)//читаем память от начала и до конца
	{
		while(ADDR % EEPROM_BLOCK_SIZE != 0)//ищем ячейку, кратную размеру блока
		{
			if(ADDR >= 0x1FFF)//если метка не найдена, возвращаем значение 0 - пишем EEPROM с нулевой ячейки
			{
				char arr1[] = "Mark not found. Srarting with 0x00.\r\n";
				sprintf(arr1, "Mark not found. Srarting with 0x00.\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)arr1, sizeof(arr1), 100);
				return 0;
			}
				ADDR++;//ищем следующую ячейку
		}
		for(uint8_t i = 0; i < EEPROM_BLOCK_SIZE; i++)//проверяем метку начала записи. По всему блоку должны быть 0xFF
		{
			if(eepromREAD_byte(ADDR) == 0xFF)
			{
				ADDR++;
				counter++;//счетчик одинаковых байт
			}else{
				counter = 0;
				break;
			}
		}
		if(counter == EEPROM_BLOCK_SIZE)// если счетчик одинаковых байт == 64(размеру блока) - метка найдена, выходим из цикла
		{
			break;
		}
	}
		if(service_mode == 128)//выводим адрес метки в UART
		{
			static char arr1[14];//массив для отправки в UART
			memset(arr1, 0x20, sizeof(arr1));
			sprintf(arr1,"Mark: %u\r", ADDR);
			HAL_UART_Transmit_IT(&huart2, (uint8_t*) arr1, strlen(arr1));
			memset(arr1, 0x20, sizeof(arr1));
		}
	return ADDR;
}



uint32_t eeprom_read_mil(void)
{
	uint32_t mil = 0;
	eepromREAD_block(eepromFIND_SPACE());
	mil = atoi((char*)block.total_mileage);
	clear_eepromBUFFER();
	return mil;
}
uint32_t eeprom_read_pmil(void)
{
	uint32_t pmil = 0;
	eepromREAD_block(eepromFIND_SPACE());
	pmil = atoi((char*)block.subtotal_mileage);
	clear_eepromBUFFER();
	return pmil;
}
void eeprom_write_totalmil(uint32_t mil)
{
	eepromREAD_block(eepromFIND_SPACE());
	sprintf((char*)block.total_mileage, "%d", mil);
	eepromWRITE_block(eepromFIND_SPACE());
	clear_eepromBUFFER();
}
void eeprom_write_subtotalmil(uint32_t subtotalmil)
{
	eepromREAD_block(eepromFIND_SPACE());
	sprintf((char*)block.subtotal_mileage, "%d", subtotalmil);
	eepromWRITE_block(eepromFIND_SPACE());
	clear_eepromBUFFER();
}
		


