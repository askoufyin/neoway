#include "main.h"
//#include "uart.h"

/* M95040 SPI EEPROM defines */
#define EEPROM_WREN  0x06  /*!< Write Enable */
#define EEPROM_WRDI  0x04  /*!< Write Disable */
#define EEPROM_RDSR  0x05  /*!< Read Status Register */
#define EEPROM_WRSR  0x01  /*!< Write Status Register */
#define EEPROM_READ  0x03  /*!< Read from Memory Array */
#define EEPROM_WRITE 0x02  /*!< Write to Memory Array */

#define EEPROM_WIP_FLAG        0x01  /*!< Write In Progress (WIP) flag */

#define EEPROM_PAGESIZE        32    /*!< Pagesize according to documentation */
#define EEPROM_BUFFER_SIZE     32    /*!< EEPROM Buffer size. Setup to your needs */

#define EEPROM_SIZE 0x2000//������������ ������ ������ ������ � ������
#define EEPROM_BLOCK_SIZE 64//������� ������ ������

#define EEPROM_BLOCK_QUANTITY (EEPROM_SIZE / EEPROM_BLOCK_SIZE)//���������� ����� ������ ��� ������� ������ 32 �����

#define EEPROM_CS_HIGH()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define EEPROM_CS_LOW()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
//#define WriteProtect_GPIO_Port GPIOC
//#define WriteProtect_Pin GPIO_PIN_4



#define TOTAL_MILEAGE_BYTES 24 //24 ������� ��� �������
#define SUBTOTAL_MILEAGE_BYTES 24 //24 ������� ��� �������������� �������
#define TIME_BYTES 16 // 16 �������� ��� UNIX TIME



//----------------------------��������� ����� ������-----------------------------
typedef struct
{
	uint8_t total_mileage[TOTAL_MILEAGE_BYTES];
	uint8_t subtotal_mileage[SUBTOTAL_MILEAGE_BYTES];
	uint8_t unix_time[TIME_BYTES];
}memory;

//���� ������ ��� eepromError_Handler
typedef enum
{
	ERROR_RW,
	ERROR_UART,
	ERROR_ACCEL,
	ERROR_N720,
}error;

void eepromError_Handler(error);
/*
��� ������ ���������� ����� ���������� ��������� 
�������� ������������ ����� ��������� ������ ������
*/
//typedef enum
//{
//	READ,
//	WRITE,
//}find_mark;

//memory block;
//memory *p_block = &block;


//-----------------------------���������-----------------------------------------
void eepromWREN(void);//��������� ������
uint8_t eepromRDSR(void);//������ ������� �������
uint8_t eepromREAD_byte(uint16_t ADDR);//������ �� FLASH, ���� ����� ������, �������� ������ �� ���
void eepromREAD_all(void);//������ ��� ������ 
void eepromWRITE_byte(uint16_t ADDR, uint8_t DATA);//������ �� FLASH
void eepromERASE(void);//������� ��� ������
uint16_t eepromFIND_SPACE_byte(void); //���� ��������� ����� ��� ������, ���� 3 ������ ������ 0�00 - �������. ���������� ����� ������ ������. ���� ������������ �������� 0xFFFF - ���������� ����� ��� ������ ���

void eepromWRSR(uint8_t REG);//���������� ������� �������. REG �������� �������� ������� ����� ��������

void eepromREAD_block(uint16_t ADDR);//������ ���� � 32 ����� 
void blockREAD(uint16_t ADDR);
//---------------------------��������� ������-----------------------------------
void eepromREAD_block (uint16_t ADDR);
void eepromWRITE_block(uint16_t ADDR);
uint16_t eepromFIND_SPACE(void);
void clear_eepromBUFFER(void);
//--------------������/������ �������----------------
uint32_t eeprom_read_mil(void);
uint32_t eeprom_read_pmil(void);
void eeprom_write_totalmil(uint32_t mil);
void eeprom_write_subtotalmil(uint32_t subtotalmil);
//------------------------������� �������� ������---------------------------------
//void main_task(void);
void task_parse(void);//��������� ������


