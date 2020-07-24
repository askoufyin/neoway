#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "gps_greatcircle.h"
#include "nmea_parser.h"
#include "uart.h"

#define PI 3.141592653589793
#define RADIUS 6372795

//extern gpgga_struct_type_def gpgga_struct;//отсюда берем данные для рассчета
extern gprmc_struct_type_def gprmc_struct;
extern gps_struct_typedef gps_struct;

typedef struct
{
	float lat_deg;
	float lon_deg;
}gps_struct_type_def;

gps_struct_type_def gps_pos;
gps_struct_type_def gps_pos_old;


//uint8_t gpgga_init(void)
//{
//	gps_pos_old.lat_deg = atof(gpgga_struct.latitude);
//	if(gpgga_struct.ns == 'N')gps_pos.lat_deg *= 1;
//	else if(gpgga_struct.ns == 'S')gps_pos.lat_deg *= (-1);
//	else return 0;
//		
//	gps_pos_old.lon_deg = atof(gpgga_struct.longitude);
//	if(gpgga_struct.we == 'E')gps_pos.lon_deg *= (-1);
//	else if(gpgga_struct.we == 'W')gps_pos.lon_deg *= 1;
//	else return 0;
//	
//	return 1;
//}

//-----------для gpgga-------------
//static uint8_t convert_str_to_float(void)
//{
//	if(gpgga_struct.GPS_quality == '0')return 0;//если обрыв антенны или нет GPS_fix
//	memset(&gps_pos, 0x00, sizeof(gps_pos));
//	/*
//		перевод строки в float и коррекция 
//		в зависимости от полушария и полюса
//	*/
//	gps_pos.lat_deg = atof(gpgga_struct.latitude);
//	if(gpgga_struct.ns == 'N')gps_pos.lat_deg *= 1;
//	else if(gpgga_struct.ns == 'S')gps_pos.lat_deg *= (-1);

//	gps_pos.lon_deg = atof(gpgga_struct.longitude);
//	if(gpgga_struct.we == 'E')gps_pos.lon_deg *= (-1);
//	else if(gpgga_struct.we == 'W')gps_pos.lon_deg *= 1;

//	return 1;
//}
//------------для gprmc-------------
//uint8_t gprmc_init(void)
//{
//	gps_pos_old.lat_deg = atof(gprmc_struct.latitude);
//	if(gprmc_struct.ns == 'N')gps_pos.lat_deg *= 1;
//	else if(gprmc_struct.ns == 'S')gps_pos.lat_deg *= (-1);
//	else return 0;
//		
//	gps_pos_old.lon_deg = atof(gprmc_struct.longitude);
//	if(gprmc_struct.ew == 'E')gps_pos.lon_deg *= (-1);
//	else if(gprmc_struct.ew == 'W')gps_pos.lon_deg *= 1;
//	else return 0;
//	
//	return 1;
//}


//static uint8_t convert_str_to_float(void)
//{
//	if(gprmc_struct.status_of_position_fix == 'V')return 0;
//	memset(&gps_pos, 0x00, sizeof(gps_pos));
//	/*
//		перевод строки в float и коррекция 
//		в зависимости от полушария и полюса
//	*/
//	//широта
//	gps_pos.lat_deg = atof(gprmc_struct.latitude);
//	if(gprmc_struct.ns == 'N')gps_pos.lat_deg *= 1;
//	else if(gprmc_struct.ns == 'S')gps_pos.lat_deg *= (-1);
//	
//	
//	//долгота
//	gps_pos.lon_deg = atof(gprmc_struct.longitude);
//	if(gprmc_struct.ew == 'E')gps_pos.lon_deg *= 1;
//	else if(gprmc_struct.ew == 'W')gps_pos.lon_deg *= (-1);
//	
//	return 1;
//}

uint8_t zero_point_init(void)
{
	n720_uart_rcv_package_it();//!!!!!!!!
	if(GPRMC_parser() == 0)return 0;//запрашиваем точку отсчета
	memset(&gps_pos_old, 0x00, sizeof(gps_pos_old));
	gps_pos_old.lat_deg = atof(gprmc_struct.latitude);
//	if(gps_pos_old.lat_deg == 0)return 0;
	gps_pos_old.lon_deg = atof(gprmc_struct.longitude);
//	if(gps_pos_old.lon_deg == 0)return 0;
	memset(&gps_pos, 0x00, sizeof(gps_pos));
	return 1;
}

static uint8_t convert_str_to_float(void)
{
	memset(&gps_pos, 0x00, sizeof(gps_pos));
	gps_pos.lat_deg = atof(gps_struct.latitude);
	if(gps_pos.lat_deg == 0)return 0;
	gps_pos.lon_deg = atof(gps_struct.longitude);
	if(gps_pos.lon_deg == 0)return 0;
	return 1;
}

//считаем радианы
uint32_t get_distance(void)
{
	if(convert_str_to_float() == 0)return 0;
	float lat1, lon1, lat2, lon2;
	//считаем радианы
	lat1 = gps_pos.lat_deg * PI / 180;
	lon1 = gps_pos.lon_deg * PI / 180;
	lat2 = gps_pos_old.lat_deg * PI / 180;
	lon2 = gps_pos_old.lon_deg * PI / 180;
	//косинусы и синусы широт и долгот
	float cl1 = cos(lat1);
	float cl2 = cos(lat2);
	float sl1 = sin(lat1);
	float sl2 = sin(lat2);
	float delta = lon2 - lon1;
	float cdelta = cos(delta);
	float sdelta = sin(delta);
	//вычисление длины большого круга
	float y = sqrt(pow(cl2*sdelta,2)+pow(cl1*sl2-sl1*cl2*cdelta,2));
	float x = sl1*sl2+cl1*cl2*cdelta;
	float ad = atan2(y,x);
	float dist = ad*RADIUS;
	gps_pos_old = gps_pos;
	return dist;
}


//float get_distance(void)
//{
//	convert_str_to_float();//заполняем поля структур получеными от n720 данными
//	calc_radians();
//	calc_central_angle();
//	if(gps_pos_old_empty == true)
//	{
//		gps_pos_old = gps_pos;
//		gps_pos_old_empty = false;
//		return 0;
//	}
//	calculate_distance();
//	gps_pos_old = gps_pos;
//	return gps_pos.distance;
//}
