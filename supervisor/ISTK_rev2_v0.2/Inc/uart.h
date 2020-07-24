#include "main.h"

#define BUFF_LENGTH 128 //буфер передаваемых/принимаемых данных


typedef enum
{
	transmit_cplt,
	transmit_busy,
	transmit_error,
}transmit_package_status_type_def;


typedef enum
{
	rcv_cplt,
	rcv_data_ready,
	rcv_busy,
	rcv_error,
}rcv_package_status_type_def;


void n720_uart_transmit_package_dma(void);
void n720_uart_transmit_str_dma(char* str);
void n720_uart_rcv_package_it(void);
void n720_clear_rx(void);
void copy_n720_to_service(void);

uint8_t service_parser(void);//парсер сервисных команд

void service_uart_transmit_package_dma(void);
void service_uart_transmit_str_dma(char* str);
void service_uart_rcv_package_it(void);
void service_clear_rx(void);
