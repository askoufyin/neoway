#include "main.h"

#define EEPROM_WREN  0x06  /*!< Write Enable */
#define EEPROM_WRDI  0x04  /*!< Write Disable */
#define EEPROM_RDSR  0x05  /*!< Read Status Register */
#define EEPROM_WRSR  0x01  /*!< Write Status Register */
#define EEPROM_READ  0x03  /*!< Read from Memory Array */
#define EEPROM_WRITE 0x02  /*!< Write to Memory Array */

#define EEPROM_WIP_FLAG        0x01  /*!< Write In Progress (WIP) flag */

//#define EEPROM_PAGESIZE        32    /*!< Pagesize according to documentation */
#define EEPROM_BUFFER_SIZE     32    /*!< EEPROM Buffer size. Setup to your needs */

#define EEPROM_SIZE 8192//максимальный размер адреса пам€ти в байтах
#define EEPROM_BLOCK_SIZE 64//размеер €чейки пам€ти

#define EEPROM_BLOCK_QUANTITY (EEPROM_SIZE / EEPROM_BLOCK_SIZE)//количество €чеек пам€ти при размере €чейки 32 байта

#define EEPROM_CS_HIGH()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define EEPROM_CS_LOW()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)


#define TOTAL_MILEAGE_BYTES 24 //24 символа дл€ пробега
#define SUBTOTAL_MILEAGE_BYTES 24 //24 символа дл€ промежуточного пробега
#define TIME_BYTES 6 // 9 символов дл€ времени 	ччммсс
#define DATE_BYTES 6 // 6 символов дл€ даты			ггммдд
#define CRC_BYTES 1
#define RESERVED_BYTES 2
//----------------------------структура €чеек пам€ти-----------------------------
typedef struct
{
	uint8_t total_mileage[TOTAL_MILEAGE_BYTES];
	uint8_t subtotal_mileage[SUBTOTAL_MILEAGE_BYTES];
	uint8_t time[TIME_BYTES];
	uint8_t date[DATE_BYTES];
	uint8_t checksum;
	uint16_t cell_num;
}memory;

//uint8_t eeprom_write_byte(uint16_t addr, uint8_t data);
//uint8_t eeprom_read_byte(uint16_t addr);

uint32_t eeprom_read_totalmil(void);//получение общего пробега
uint32_t eeprom_read_subtotalmil(void);//получение промежуточного пробега
void eeprom_write_mileage(uint32_t total_mil, uint32_t subtotal_mil);//запись сразу всего пробега
void eeprom_write_totalmil(uint32_t total_mil);//запись полного пробега
void eeprom_write_subtotalmil(uint32_t subtotal_mil);//запись промежуточного порбега

uint8_t eeprom_read_block(uint8_t block_number);
uint8_t eeprom_write_block(uint8_t block_number);
uint8_t eeprom_find_mark(void);
void eeprom_init(void);
void eeprom_erase(void);//очистка eeprom
void eeprom_send_block_to_uart(void);//вычитываем блок данных, отправл€ем в UART
void eeprom_write_data(uint32_t total_mil, uint32_t subtotal_mil, char * date, char * time);


void eeprom_read_all(void);

uint8_t eeprom_read_byte(uint16_t addr);
uint8_t eeprom_write_byte(uint16_t addr, uint8_t data);
uint8_t eeprom_rdsr(void);
