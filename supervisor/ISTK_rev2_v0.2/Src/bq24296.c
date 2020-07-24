#include "bq24296.h"
#include <stdio.h>
#include <string.h>


extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

typedef enum 
{
	not_charging,
	pre_charge,
	fast_charging,
	charge_termination_done,
	power_good,
	not_power_good,
	thermal_normal,
	thermal_regulation
}bq24296_status_type_def;

bq24296_status_type_def bq24296_status;

enum bq24296_check_status
{
	check_power_good,
	check_charging,
};

enum error_type
{
	who_am_i_error,
	connection_error,
	bq24296_fault,
};

enum REG_01_CFG
{
	SET_OTG_CONFIG,
	SET_CHG_CONFIG
};

void bq24296_error(uint8_t error_type)
{
	switch (error_type)
	{
		case who_am_i_error:
		{
			char err_msg[32] = {0};
			sprintf(err_msg, "BQ24296: sensor detection error\r");
			HAL_UART_Transmit(&huart2, (uint8_t*)err_msg, strlen(err_msg), 100);
			break;
		}
		case connection_error:
		{
			char err_msg[32] = {0};
			sprintf(err_msg, "BQ24296: connection error\r");
			HAL_UART_Transmit(&huart2, (uint8_t*)err_msg, strlen(err_msg), 100);
			break;
		}
		case bq24296_fault:
		{
			char fault_msg[32];
			sprintf(fault_msg, "bq24296_fault\r");
			HAL_UART_Transmit(&huart2, (uint8_t*)fault_msg, strlen(fault_msg), 100);
	//while(1);
		}
	}
}

// 0110 1000 - addr (0 - write, 1 - read)

static uint8_t readReg(uint16_t reg_addr)//dev_addr if AD0 = 0 0x68, if AD0 = 1 0x69 
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t ret = 0;
	status = HAL_I2C_Mem_Read(&hi2c2, BQ24296_DEVICE_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, &ret, 1, 0x10000);
	if(status != HAL_OK)bq24296_error(connection_error);
	return ret;
}

static void writeReg(uint16_t reg_addr, uint8_t data)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c2, BQ24296_DEVICE_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0x10000);
	if(status != HAL_OK)bq24296_error(connection_error);
}

uint8_t whoAmI(void)
{
	uint8_t ret = readReg(WHO_AM_I);
//	if(ret != WHO_AM_I_REG)Error();
	return ret;
}

//конфигурация регистра 00
void set_input_source_control_reg00(void)
{
	uint8_t reg = 0;
	//writeReg(INPUT_SOURCE_CONTROL_REG, reg);
	//reg = readReg(INPUT_SOURCE_CONTROL_REG);
	reg ^= SET_VINDPM_4360mV ;//входное напряжение 4,36v
	reg ^= SET_IINLIM_1500mA;//какой выбрать IINLIM?
	writeReg(INPUT_SOURCE_CONTROL_REG, reg);
}

//конфигурация регистра 01
void set_power_on_configuration_reg01(uint8_t REG_01_CFG)
{
	uint8_t reg = 0;
	
	reg &= ~POWER_ON_CONFIGURATION_RESET;//бит сброса этого регистра устанавливаем в 0 (не сбрасываем)
	reg &= ~I2C_WATCHDOG_TIMER_RESET;//watchdog timer reset оставляем 0
	/*
					Биты OTG_CONFIG и CHG_CONFIG
	--------не могут быть установлены -----------
					единовременно
	*/
	if(REG_01_CFG == SET_OTG_CONFIG)
	{
		reg &= OTG_CONFIG;//1
		reg &= ~CHG_CONFIG;//0
	}
	else if(REG_01_CFG == SET_CHG_CONFIG)
	{	
		reg &= ~OTG_CONFIG;//0
		reg &= CHG_CONFIG;//разрешаем заряд батареи, записываем 1
	}
	//--------------------------------------------
	reg &= SET_SYS_MIN;
	reg &= SET_BOOST_LIM;
	writeReg(POWER_ON_CONFIGURATION_REG, reg);
}

//конфигурация регистра 02
void set_charge_current_control_reg02(void)
{
	uint8_t reg = 0;
	reg ^= SET_ICHG_2000mA;//предел тока быстрой зарядки (2а)
	reg &= ~SET_BCOLD;
	reg &= ~SET_FORCE_20PCT;
	writeReg(CHARGE_CURRENT_CONTROL_REG, reg);
}

//конфигурация регистра 03
void set_pre_charge_termination_current_control_reg03(void)
{
	uint8_t reg = 0;
	reg ^= SET_PRE_CHARGE_CURRENT_LIMIT_128mA;//ток предзаряда 128mA
	reg ^= SET_PRE_CHARGE_TERMINATION_LIMIT_128mA;//ток завершения предзаряда 
	writeReg(PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REG, reg);
}

//конфигурация регистра 04
void set_charge_voltage_control_reg04(void)
{
	uint8_t reg = 0;
	reg ^= SET_VREG_4208mV;
	reg ^= SET_BATLOWV_3000mV;
	reg ^= SET_VRECHG_300mV;
	writeReg(CHARGE_VOLTAGE_CONTROL_REG, reg);
}

//конфигурация регистра 05
void set_charge_termination_timer_control_reg05(void)
{
	uint8_t reg = 0;
	reg ^= EN_TERM;
	reg ^= SET_I2C_WATCHDOG_40sec;
	reg ^= EN_TIMER;
	reg ^= SET_CHG_TIMER_8hrs;
	writeReg(CHARGE_TERMINATION_TIMER_CONTROL_REG, reg);
}

//конфигурация регистра 06
void set_boost_voltage_thermal_regulation_control_reg06(void)
{
	uint8_t reg = 0;
	reg ^= SET_BOOSTV_4550mV;
	reg ^= SET_BHOOT_33pcnt_OF_REGN_OR_55degC;
	reg ^= SET_TREG_120deg;
	writeReg(BOOST_VOLTAGE_THERMAL_REGULATION_CONTROL_REG, reg);
}

//конфигурация регистра 07
void set_misc_operation_control_reg07(void)
{
	
}

//проверяем статус устройства

//uint8_t bq2496_check_stat_reg08(void)
//{
//	uint8_t reg = readReg(SYSTEM_STATUS_REG);
//	
//}

uint8_t bq2496_check_stat_reg08(uint8_t bq24296_check_status)
{
	switch(bq24296_check_status)
	{
		case check_power_good:
		{
			uint8_t reg = readReg(SYSTEM_STATUS_REG);
			reg = reg << 5;
			reg = reg >> 7;
			if(reg == 1)
			{
				bq24296_status = power_good;
			}else{
				bq24296_status = not_power_good;
				//return bq24296_status;
			}
			break;
		}
		case check_charging:
		{
			uint8_t reg = readReg(SYSTEM_STATUS_REG);
			reg = reg << 2;
			reg = reg >> 6;
			
			if(reg == 0x00)//not charging
			{
				bq24296_status = not_charging;
				//return bq24296_status;
			}
			else if(reg == 0x01)//pre-charge
			{
				bq24296_status = pre_charge;
				//return bq24296_status;
			}
			else if(reg == 0x02)//fast charging
			{
				bq24296_status = fast_charging;
				//return bq24296_status;
			}
			else if(reg == 0x03)//charge terminftion done
			{
				bq24296_status = charge_termination_done;
				//return bq24296_status;
			}
			break;
		}
	}
	return bq24296_status;
}

//получаем код ошибки
uint8_t bq2496_get_error_reg09(void)
{
	uint8_t reg = readReg(NEW_FAULT_REG);
	if(reg != 0x00)
	{
		bq24296_error(bq24296_fault);
	}
	return reg;
}

//инициализация
void bq2496_ini(void)
{
	if(whoAmI() == WHO_AM_I_REG)
	{
		set_input_source_control_reg00();
		set_power_on_configuration_reg01(SET_CHG_CONFIG);
		set_charge_current_control_reg02();
		set_charge_voltage_control_reg04();
		set_charge_termination_timer_control_reg05();
		set_boost_voltage_thermal_regulation_control_reg06();
		set_misc_operation_control_reg07();
	}else{
		bq24296_error(who_am_i_error);
	}
}

//проверка 24в на шине питания
		//сигнал 24в инверсный


uint8_t check24v(void)
{
//	check24v_stat_type_def stat24v;
	if(HAL_GPIO_ReadPin(CTRL_24V_GPIO_Port, CTRL_24V_Pin)!=GPIO_PIN_SET)//сигнал инверсный
	{
		//24в на линии
		//stat24v = on;
		return 1;
	}else{
		//24в отсутствует
		//отправляем информацию на N720 об окончании работы
		char str[32];
		sprintf(str, "AT+SLEEP");
		HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
		//stat24v = off;
		return 0;
	}
}

//uint8_t whoiam(void)
//{
//	uint8_t ret;
//	HAL_I2C_Mem_Read(&hi2c1, BQ24296_DEVICE_ADDRESS, WHO_I_AM_REG, I2C_MEMADD_SIZE_8BIT, &ret, sizeof(ret), 1000);
//	return ret;
//}	
