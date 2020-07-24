#include "indication.h"

/*
	Индикация осуществляется битовыми масками
	Каждую итерацию в основном цикле
	Битовая маска смещается на 1 бит 
	И в зависимости от получившегося значения (0 или !0)
	Выключаем или включаем светодиод
	
	Режим индикации выбирается исходя из типа устройства
	С батарейным питанием или без
	Это определяется наличием сигнала на пине РВ9(i2c1 SDA)
	Если устройство с батарейным питанием, на этом пине сохраняется высокий уровень
	Если без батарейного питания, то уровень на РВ9 - низкий
	
	Индикация осуществляется светодиодами FAULT и POWER
	Светодиод FAULT указывает на наличие ошибок 
*/

	//битовая маска постоянно смещается на lsh_counter бит
	#define MASK 0x01U
	
indication_error_struct_typedef indication_err;//отсюда считываем информацию об ошибках и выполняем действия

extern TIM_HandleTypeDef htim6;
extern uint8_t n720_on;

static uint8_t lsh_counter = 0;//сдвиговый счетчик
static uint8_t switcher = 0;//счетчик событий идикации
static uint32_t indication_tick = 0;//переменная для соблюдения таймингов морганий

void in_handler(void);//поток

/*
	тип устройства
	с батареей; без батарем
*/
enum 
{
	mod_bat,
	mod_nobat,
}device_type;

void indication_init(void)
{
	indication_err.sim_error = 0;
	indication_err.charger_error = 0;
	indication_err.battery_error = 0;
	HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_SET);//зажигаем power при инициализации
	/*
		вычитываем состояния пина i2c на контроллере заряда
		если пин подтянут наверх значит устройство имеет контроллер заряда
		в зависимости от этого выбираем режим индикации
	*/
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
	{
		device_type = mod_bat;
	}else{
		device_type = mod_nobat;
	}
	
}

void in_handler(void)
{
//-------------------индикация для устройства с батарейным питанием-------------------		
		if(device_type == mod_bat)//если устройство оборудовано батареей индицируем события: ошибка сим карты, ошибка контроллера заряда, ошибка батарем
		{
			if(HAL_GPIO_ReadPin(CTRL_24V_GPIO_Port, CTRL_24V_Pin) == GPIO_PIN_SET)//если питание от батареи - моргаем светодиодом power(пин инверсный)
			{
				indication_err.battery_discarging = 1;
				HAL_GPIO_TogglePin(POWER_LED_GPIO_Port, POWER_LED_Pin);
			}else{
				indication_err.battery_discarging = 0;
				HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_SET);
			}
			if(n720_on == 0)//n720 error если neoway не поднялся
			{
				HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
				return;
			}
			if(indication_err.sim_error == 0 && indication_err.charger_error == 0 && indication_err.battery_error == 0)//если неисправности не найдены гасим FAULT, выходим из обработчика
			{
				HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
				return;
			}
			switch(switcher)
			{
				case(0)://ошибка сим карты
				{
					if(indication_err.sim_error == 1 && lsh_counter < 8)//если есть сообщение об ошибке и не все биты маски отработаны
					{
						volatile uint8_t msk;
						msk = MASK << lsh_counter;//сдвигаем едиицу на 1 бит влево
						lsh_counter++;//инкрементируем сдвиговый счетчик
						volatile uint8_t res_sim;//результат битовой операции 
						res_sim = SIM_ERROR_MSK & msk;//AND сдвинутую на один бит единицу и битовую маску индикации
						if(res_sim != 0)//если результат предыдущей операции не 0 - зажигаем светодиод
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
							return;
						}
						else if(res_sim == 0)//если результат операции 0 - гасим светодиод
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
							return;
						}
					}else{//если сообщения об ошибке нет или отработаны все биты предыдущей битовой маски - переходим в блок обработки индикации для следующего события
						switcher++;
						lsh_counter = 0;
						return;
					}
				}
				case(1)://ошибка зарядного устройства
				{
					if(indication_err.charger_error == 1 && lsh_counter < 8)
					{
						volatile uint8_t msk;
						msk = MASK << lsh_counter;
						lsh_counter++;
						volatile uint8_t res_chg;
						res_chg = CHARGER_ERROR_MSK & msk;
						if(res_chg != 0)
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
							return;
						}
						else if(res_chg == 0)
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
							return;
						}
					}else{
						switcher++;
						lsh_counter = 0;
						return;
					}
				}
				case(2)://ошибка батареи
				{
					if(indication_err.battery_error == 1 && lsh_counter < 8)
					{
						volatile uint8_t msk;
						msk = MASK << lsh_counter;
						lsh_counter++;
						volatile uint8_t res_bat;
						res_bat = BATTERY_ERROR_MSK & msk;//0B11010100U;
						if(res_bat !=  0)
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
							return;
						}
						else if(res_bat == 0)
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
							return;
						}
					}else{
						switcher++;
						lsh_counter = 0;
						return;
					}
				}
				default:
				{
					switcher = 0;
					lsh_counter = 0;
					return;
				}
			}
		}
		
//-------------------индикация для устройства без батарейного питания-------------------		
		if(device_type == mod_nobat)//если устройство не оборудовано батареей индицируем события: ошибка сим карты
		{
			if(n720_on == 0)//n720 error если neoway не поднялся
			{
				HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
				return;
			}
			if(indication_err.sim_error == 0 && indication_err.charger_error == 0 && indication_err.battery_error == 0)//если неисправности не найдены гасим FAULT, выходим из обработчика
			{
				HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
				return;
			}
			
			switch(switcher)
			{
				case(0):
				{
					if(indication_err.sim_error == 1 && lsh_counter < 8)
					{
						volatile uint8_t msk;
						msk = MASK << lsh_counter;
						lsh_counter++;
						volatile uint8_t res_sim;
						res_sim = SIM_ERROR_MSK & msk;
						if(res_sim != 0)
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
							return;
						}
						else if(res_sim == 0)
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
							return;
						}
					}else{
						switcher++;
						lsh_counter = 0;
						return;
					}
				}
				default:
				{
					switcher = 0;
					lsh_counter = 0;
					return;
				}
			}
		}

}

void indication_handler(uint32_t time_of_blink)
{
	if(HAL_GetTick() - indication_tick > time_of_blink)
	{
		in_handler();
		indication_tick = HAL_GetTick();
	}
}
