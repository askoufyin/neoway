#include "main.h"

//#define UART_BUFF_LENGTH 128 //буфер передаваемых/принимаемых данных

typedef struct
{
	char utc_time[sizeof("hhmmss.ss")];
	char latitude[sizeof("dddd.mmmmmm")];
	char ns;//N - north, S - south
	char longitude[sizeof("ddddd.mmmmmm")];
	char we;//W - west, E - east
	char GPS_quality;//0 - no fix, 1 - fix, 2 - differential fix, 3 - not valid, 6 - estimated, если 0 - антенна не подключена
	char  satellites_in_use[sizeof("xx")];
}gpgga_struct_type_def;
//$GPGGA,090737.00,3528.33,N,02441.36,E,1,05,1.4,135.1,M,0,M,,*44
//$MYGPSPOS: $GPGGA,,,,,,0,,,,,,,,*66

typedef struct
{
	char utc_time[sizeof("hhmmss.ss")];
	char status_of_position_fix;//A - valid, V - invalid
	char latitude[sizeof("dddd.mmmmmm")];
	char ns;//N- north, S - south
	char longitude[sizeof("ddddd.mmmmmm")];
	char ew;//E - east, W - west
	char speed_over_ground_knots[sizeof("999.9")];
	char track_made_good[sizeof("359.9")];
	char utc_date[sizeof("ddmmyy")];
	char magnetic_variation_degrees[sizeof("180.0")];
	char mvd_ew;//eastern/western of magnetic_variation_degrees
	char gps_quality_indicator;//A - automatic, D - DGPS, E - estimated, N - invalid
}gprmc_struct_type_def;
//$MYGPSPOS: $GPRMC,142105.00,A,5653.575952,N,03550.284210,E,0.0,144.0,020620,8.8,E,A,V*49
//$MYGPSPOS: $GPRMC,,V,,,,,,,,,,N*53

typedef struct
{
	char sn[30];
	char time_stamp[16];
	char latitude[10];
	char longitude[11];
	char status;
	//сюда складываем преобразованное время
	char date[6];
	char time[9];
}gps_struct_typedef;

uint8_t gps_fix_parser(void);//3D fix, смотрим наличие
//uint8_t GGA_parser(void);//получаем координату
uint8_t GPRMC_parser(void);//получаем координаты с датой
//uint8_t gps_parser(void);//собственный протокол. neoway постоянно шлет данные. Сдушаем, анализируем
uint8_t n720_ack_timeout(uint32_t timeout);//таймаут без delay

void n720_shutdown_cmd(void);
void at_init(void);
uint8_t info_parser(void);

