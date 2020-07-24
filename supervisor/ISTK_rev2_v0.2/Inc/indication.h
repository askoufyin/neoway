#include "main.h"

/*
	FAULT горит при инициализации, при ошибке ядра, 
	моргает при отсутствии SIM
*/


	//сигналы - моргалки 
	#define SIM_ERROR_MSK 			0x0FU		//0B11000000U
	#define CHARGER_ERROR_MSK 	0x05U		//0B11010000U
	#define BATTERY_ERROR_MSK 	0x15U		//0B11010100U

typedef struct
{
	uint8_t sim_error;
	uint8_t charger_error;
	uint8_t battery_error ;
	uint8_t battery_discarging;
}indication_error_struct_typedef;

void indication_init(void);
void indication_handler(uint32_t time_of_blink);//задаем частоту морганий
