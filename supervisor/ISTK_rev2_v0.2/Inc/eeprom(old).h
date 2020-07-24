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

#define EEPROM_SIZE 0x2000//максимальный размер адреса памяти в байтах
#define EEPROM_BLOCK_SIZE 64//размеер ячейки памяти

#define EEPROM_BLOCK_QUANTITY (EEPROM_SIZE / EEPROM_BLOCK_SIZE)//количество ячеек памяти при размере ячейки 32 байта

#define EEPROM_CS_HIGH()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define EEPROM_CS_LOW()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
//#define WriteProtect_GPIO_Port GPIOC
//#define WriteProtect_Pin GPIO_PIN_4



#define TOTAL_MILEAGE_BYTES 24 //24 символа для пробега
#define SUBTOTAL_MILEAGE_BYTES 24 //24 символа для промежуточного пробега
#define TIME_BYTES 16 // 16 символов для UNIX TIME



//----------------------------структура ячеек памяти-----------------------------
typedef struct
{
	uint8_t total_mileage[TOTAL_MILEAGE_BYTES];
	uint8_t subtotal_mileage[SUBTOTAL_MILEAGE_BYTES];
	uint8_t unix_time[TIME_BYTES];
}memory;

//типы ошибок для eepromError_Handler
typedef enum
{
	ERROR_RW,
	ERROR_UART,
	ERROR_ACCEL,
	ERROR_N720,
}error;

void eepromError_Handler(error);
/*
при поиске свободного места необходимо учитывать 
смещение относительно метки окончания записи данных
*/
//typedef enum
//{
//	READ,
//	WRITE,
//}find_mark;

//memory block;
//memory *p_block = &block;


//-----------------------------прототипы-----------------------------------------
void eepromWREN(void);//разрешаем запись
uint8_t eepromRDSR(void);//читаем регистр статуса
uint8_t eepromREAD_byte(uint16_t ADDR);//чтение из FLASH, даем адрес ячейки, получаем данные из нее
void eepromREAD_all(void);//читаем всю память 
void eepromWRITE_byte(uint16_t ADDR, uint8_t DATA);//запись во FLASH
void eepromERASE(void);//стираем всю память
uint16_t eepromFIND_SPACE_byte(void); //ищем свободное место для записи, если 3 ячейки подряд 0х00 - найдено. Возвращаем адрес пустой ячейки. Если возвращаемое значение 0xFFFF - свободного места для записи нет

void eepromWRSR(uint8_t REG);//записываем регистр статуса. REG значение регистра которое хотим записать

void eepromREAD_block(uint16_t ADDR);//читаем блок в 32 байта 
void blockREAD(uint16_t ADDR);
//---------------------------поблочная запись-----------------------------------
void eepromREAD_block (uint16_t ADDR);
void eepromWRITE_block(uint16_t ADDR);
uint16_t eepromFIND_SPACE(void);
void clear_eepromBUFFER(void);
//--------------чтение/запись пробега----------------
uint32_t eeprom_read_mil(void);
uint32_t eeprom_read_pmil(void);
void eeprom_write_totalmil(uint32_t mil);
void eeprom_write_subtotalmil(uint32_t subtotalmil);
//------------------------парсинг принятых данных---------------------------------
//void main_task(void);
void task_parse(void);//разбираем данные


