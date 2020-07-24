#include "indication.h"

/*
	��������� �������������� �������� �������
	������ �������� � �������� �����
	������� ����� ��������� �� 1 ��� 
	� � ����������� �� ������������� �������� (0 ��� !0)
	��������� ��� �������� ���������
	
	����� ��������� ���������� ������ �� ���� ����������
	� ���������� �������� ��� ���
	��� ������������ �������� ������� �� ���� ��9(i2c1 SDA)
	���� ���������� � ���������� ��������, �� ���� ���� ����������� ������� �������
	���� ��� ����������� �������, �� ������� �� ��9 - ������
	
	��������� �������������� ������������ FAULT � POWER
	��������� FAULT ��������� �� ������� ������ 
*/

	//������� ����� ��������� ��������� �� lsh_counter ���
	#define MASK 0x01U
	
indication_error_struct_typedef indication_err;//������ ��������� ���������� �� ������� � ��������� ��������

extern TIM_HandleTypeDef htim6;
extern uint8_t n720_on;

static uint8_t lsh_counter = 0;//��������� �������
static uint8_t switcher = 0;//������� ������� ��������
static uint32_t indication_tick = 0;//���������� ��� ���������� ��������� ��������

void in_handler(void);//�����

/*
	��� ����������
	� ��������; ��� �������
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
	HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_SET);//�������� power ��� �������������
	/*
		���������� ��������� ���� i2c �� ����������� ������
		���� ��� �������� ������ ������ ���������� ����� ���������� ������
		� ����������� �� ����� �������� ����� ���������
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
//-------------------��������� ��� ���������� � ���������� ��������-------------------		
		if(device_type == mod_bat)//���� ���������� ����������� �������� ���������� �������: ������ ��� �����, ������ ����������� ������, ������ �������
		{
			if(HAL_GPIO_ReadPin(CTRL_24V_GPIO_Port, CTRL_24V_Pin) == GPIO_PIN_SET)//���� ������� �� ������� - ������� ����������� power(��� ���������)
			{
				indication_err.battery_discarging = 1;
				HAL_GPIO_TogglePin(POWER_LED_GPIO_Port, POWER_LED_Pin);
			}else{
				indication_err.battery_discarging = 0;
				HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_SET);
			}
			if(n720_on == 0)//n720 error ���� neoway �� ��������
			{
				HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
				return;
			}
			if(indication_err.sim_error == 0 && indication_err.charger_error == 0 && indication_err.battery_error == 0)//���� ������������� �� ������� ����� FAULT, ������� �� �����������
			{
				HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
				return;
			}
			switch(switcher)
			{
				case(0)://������ ��� �����
				{
					if(indication_err.sim_error == 1 && lsh_counter < 8)//���� ���� ��������� �� ������ � �� ��� ���� ����� ����������
					{
						volatile uint8_t msk;
						msk = MASK << lsh_counter;//�������� ������ �� 1 ��� �����
						lsh_counter++;//�������������� ��������� �������
						volatile uint8_t res_sim;//��������� ������� �������� 
						res_sim = SIM_ERROR_MSK & msk;//AND ��������� �� ���� ��� ������� � ������� ����� ���������
						if(res_sim != 0)//���� ��������� ���������� �������� �� 0 - �������� ���������
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
							return;
						}
						else if(res_sim == 0)//���� ��������� �������� 0 - ����� ���������
						{
							HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
							return;
						}
					}else{//���� ��������� �� ������ ��� ��� ���������� ��� ���� ���������� ������� ����� - ��������� � ���� ��������� ��������� ��� ���������� �������
						switcher++;
						lsh_counter = 0;
						return;
					}
				}
				case(1)://������ ��������� ����������
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
				case(2)://������ �������
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
		
//-------------------��������� ��� ���������� ��� ����������� �������-------------------		
		if(device_type == mod_nobat)//���� ���������� �� ����������� �������� ���������� �������: ������ ��� �����
		{
			if(n720_on == 0)//n720 error ���� neoway �� ��������
			{
				HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_SET);
				return;
			}
			if(indication_err.sim_error == 0 && indication_err.charger_error == 0 && indication_err.battery_error == 0)//���� ������������� �� ������� ����� FAULT, ������� �� �����������
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
