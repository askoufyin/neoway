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
����, ����������� � �������� ��� ��������� ���������� �� UART

*/




//#include "stm32f0xx_hal.h"





//-----------------WRITE ENABLE(WREN)-����������� ����� ������ �������� WRITE � WRSR---------------
void eepromWREN(void)
{
	uint8_t cmdWREN[1] = {EEPROM_WREN};
	
	EEPROM_CS_LOW();
	HAL_SPI_Transmit(&hspi1, cmdWREN, 1, 0x64);
	EEPROM_CS_HIGH();
}

//-------------------������ ������� �������(RDSR)-------------------------------------------------
uint8_t eepromRDSR(void)
{
	uint8_t cmdRSDR[1] = {EEPROM_RDSR};//RDSR command
	uint8_t resRSDR[2];
	
	EEPROM_CS_LOW();//EEPROM_CS_LOW();//set down cs //����������� nss, RESET - ��� ������������ ����������, SET ��� ���������
		
	HAL_SPI_TransmitReceive(&hspi1, cmdRSDR, resRSDR, 2, 0x64);//�������� 1 ���� �������, 2 ���� ��������� �����

	EEPROM_CS_HIGH();	
	return resRSDR[1];//������ ���� ���������� ���������� �������, � EEPROM ������, ������� ���������� ������ ���� � ������� � ����� �����
}

//----------------READ FROM MEMORY ARRAY (READ)-������ �� ������-------------------------------------
//uint8_t eepromREAD(uint8_t MSB_ADDR, uint8_t LSB_ADDR)
uint8_t eepromREAD_byte(uint16_t ADDR)
{
	uint8_t cmdRead_byte[3];
	cmdRead_byte[0] = EEPROM_READ;//������� "������"
	cmdRead_byte[1] = ADDR >> 8;//������� ���� ������ ������
	cmdRead_byte[2] = ADDR;//������� ���� ������ ������
	uint8_t resRead[4];//�����
//	HAL_StatusTypeDef status;
	
	EEPROM_CS_LOW();
	
	HAL_SPI_TransmitReceive(&hspi1, cmdRead_byte, resRead, 4, 0x64);

	EEPROM_CS_HIGH();
	return resRead[3];//������ ��� ������ 3 ����� EEPROM ������ (��. ���������� ����������)
}

//---------------WRITE TO MEMORY ARRAY (WRITE)-������ � ������--------------------------------------
void eepromWRITE_byte(uint16_t ADDR, uint8_t DATA)//
{
	uint8_t a = 0;//���-�� ������� ������ � ������
	do{
	uint8_t cmdWRITE_byte[4];
	cmdWRITE_byte[0] = EEPROM_WRITE;
	cmdWRITE_byte[1] = ADDR >> 8;//������ 8 ��� 16 ������� ������
	cmdWRITE_byte[2] = ADDR;//������ 8 ��� 16 ������� ������
	cmdWRITE_byte[3] = DATA;//������������ ������
	
	eepromWREN();
		
	EEPROM_CS_LOW();
	
	HAL_SPI_Transmit(&hspi1, cmdWRITE_byte, 4, 0x64);
	
	EEPROM_CS_HIGH();
	
	while(eepromRDSR() != 0x00)//���� ���� ���������� ������ 5ms (������� ������� ������ ���� 0�00)
	{
		
	}
	a++;
	}while(eepromREAD_byte(ADDR) != DATA && a <= 3);//���������� ���������� ������
	if(eepromREAD_byte(ADDR) != DATA)//���� ������������ ������ �� ������������� ������������
	{
	eepromError_Handler(ERROR_RW);//������ � ������ ������/������
	}else{
	return;
	}
}

	
//----------------------------------------------------------------
//	if (eepromREAD(ADDR) == DATA)//���������� ���������� ������
//	{
//											//���� ������ ��������
//	}else{							//���� ������ �� ��������
//		uint8_t a = 0;
//		do{								//�������������� 2 ����
//			uint8_t cmdWRITE[4];
//			cmdWRITE[0] = EEPROM_WRITE;
//			cmdWRITE[1] = ADDR >> 8;//������ 8 ��� 16 ������� ������
//			cmdWRITE[2] = ADDR;//������ 8 ��� 16 ������� ������
//			cmdWRITE[3] = DATA;//������������ ������
//			EEPROM_CS_LOW();
//			HAL_SPI_Transmit(&hspi1, cmdWRITE, 4, 0x64);
//			EEPROM_CS_HIGH();
//			while(eepromRDSR() != 0x00)//���� ���� ���������� ������ (������� ������� ������ ���� 0�00)
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


//--------------------------����� ������� ������� (WRSR)---------------------------------------
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
	
//--------------------���� ��������� ����� ��� ������------------------------------------------	
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
//-----------------------������� ��� ������----------------------------------------------------
void eepromERASE(void)
{
//	for(uint16_t i = 0xFF; i <= EEPROM_SIZE; i++)//!!!!!!!!!!!!!!!!!!!
	for(uint16_t i = 0x00; i <= EEPROM_SIZE; i++)
	{
		eepromWREN();
		eepromWRITE_byte(i, 0x00);
	}
}
//---------------------------������ ��� ������--------------------------------------------------
void eepromREAD_all(void)
{
uint16_t i;//������� ������� 
char arr1[18];
memset(arr1, 0x20, 18);
	
for(i = 0x0000; i < EEPROM_SIZE; i++)
	{ 
		uint8_t b = eepromREAD_byte(i);//������ � ������
		sprintf(arr1, "%0x\t%0x\r\n", i, b);
		HAL_UART_Transmit(&huart2, (uint8_t*)arr1, 9, 0xFFFF);
		memset(arr1, 0x20, 18);	
		IWDG->KR = 0x0000AAAAU;//���������� ������
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
//-----------------------��������� ����� ������----------------------------------

static memory block;
memory *p_block = &block;


	//---------------------������_�����_EEPROM------------------------------------
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
	

//--------------------������------------------------------------------------
void eepromREAD_block (uint16_t ADDR)//�������
{
	if(ADDR != 0)
	{
		ADDR = ADDR - (EEPROM_BLOCK_SIZE * 2); //�������� ����� � �������� ������
	}
	for(uint8_t i = 0; i < TOTAL_MILEAGE_BYTES; i++)//������ ������
	{
		p_block -> total_mileage[i] = eepromREAD_byte(ADDR);
		ADDR++;
	}
	for(uint8_t i = 0; i < SUBTOTAL_MILEAGE_BYTES; i++)//������ ������������� ������
	{
		p_block -> subtotal_mileage[i] = eepromREAD_byte(ADDR);
		ADDR++;
	}
	for(uint8_t i = 0; i < TIME_BYTES; i++)//������ �����
	{
		p_block -> unix_time[i] = eepromREAD_byte(ADDR);
		ADDR++;
	}
}

//---------------------������_�����_������------------------------------------------
void eepromWRITE_block(uint16_t ADDR)//������
{
	if (ADDR != 0)
	{
		ADDR = ADDR - EEPROM_BLOCK_SIZE; //�������� �����
	}
	for(uint8_t i = 0; i < TOTAL_MILEAGE_BYTES; i++)   //����� ������
	{
		eepromWRITE_byte(ADDR, p_block -> total_mileage[i]);
		ADDR++;
	}
	for(uint8_t i = 0; i < SUBTOTAL_MILEAGE_BYTES; i++)
	{
		eepromWRITE_byte(ADDR, p_block -> subtotal_mileage[i]); //����� ������������� ������
		ADDR++;
	}
	for(uint8_t i = 0; i < TIME_BYTES; i++) //����� �����
	{
		eepromWRITE_byte(ADDR, p_block -> unix_time[i]);
		ADDR++;
	}
	for(uint8_t i = 0; i < (TOTAL_MILEAGE_BYTES + SUBTOTAL_MILEAGE_BYTES + TIME_BYTES); i++)//������ ����� � ������� ������ ������ � ��������� ���(����� FF 32 ���� - ���������� ��� ������ � FF)
	{
		eepromWRITE_byte(ADDR, 0xFF);
		ADDR++;
	}
		memset(p_block -> total_mileage, 0x00, TOTAL_MILEAGE_BYTES);
		memset(p_block -> subtotal_mileage, 0x00, SUBTOTAL_MILEAGE_BYTES);
		memset(p_block -> unix_time, 0x00, TIME_BYTES);
}
//------------------------------------------------------------

//-----------------���� ����� ��� ������ ������---------------
uint16_t eepromFIND_SPACE()
{
/*
����� ��� 1 ���� � ������� ��� ����� �������� ��� 0xFF. ���� �����, ��������������� ��������� 
������ ������. ��� ������ ����� �������, ���������� ����� ������ �����. � ���� ����� ����������� ������
*/
	uint16_t ADDR = 0;
	uint8_t counter;
	counter = 0;//������� ���������� ����
	
	for(ADDR = 0; ADDR < EEPROM_SIZE; ADDR++)//������ ������ �� ������ � �� �����
	{
		while(ADDR % EEPROM_BLOCK_SIZE != 0)//���� ������, ������� ������� �����
		{
			if(ADDR >= 0x1FFF)//���� ����� �� �������, ���������� �������� 0 - ����� EEPROM � ������� ������
			{
				char arr1[] = "Mark not found. Srarting with 0x00.\r\n";
				sprintf(arr1, "Mark not found. Srarting with 0x00.\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)arr1, sizeof(arr1), 100);
				return 0;
			}
				ADDR++;//���� ��������� ������
		}
		for(uint8_t i = 0; i < EEPROM_BLOCK_SIZE; i++)//��������� ����� ������ ������. �� ����� ����� ������ ���� 0xFF
		{
			if(eepromREAD_byte(ADDR) == 0xFF)
			{
				ADDR++;
				counter++;//������� ���������� ����
			}else{
				counter = 0;
				break;
			}
		}
		if(counter == EEPROM_BLOCK_SIZE)// ���� ������� ���������� ���� == 64(������� �����) - ����� �������, ������� �� �����
		{
			break;
		}
	}
		if(service_mode == 128)//������� ����� ����� � UART
		{
			static char arr1[14];//������ ��� �������� � UART
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
		


