#include "main.h"

/*

DEFAULT SETTINGS:													INI SETTINGS:

Input voltage									4.36 V			4.36 V
Input current limit												1.5 A???
Charging voltage							4.208 V			4.208
Minimum system voltage limit	3.5 V				3.5 V
Fast charge current	limit			2.048 A			2.48 A(test) 3.072 A(release)
Pre-charge current						256 mA			128 mA
Termination current						256 mA			128��???
Battry recharge treshold			100 mV			300 mV - ����� ���������� �� ������� ������ �� ��� �������� �� ����������, ���������� VREG, �������� �����������
Boost voltage									4.998 V			4.55???
Thermal regulation treshold		120�C 			120�C 
Temperature profile						Hot/Cold		Hot/cold
Safety timer									12 hours		8 hours

*/


#define BQ24296_DEVICE_ADDRESS 0x6B<<1

#define WHO_AM_I 0x0AU
#define WHO_AM_I_REG 0x04U

//������������ �������� 00
#define INPUT_SOURCE_CONTROL_REG 0x00U//�������� ��� � ���������� �� �����
#define EN_HIZ 0x80U//�������� z ���������
#define SET_VINDPM_4360mV 0x30U//���������� ������� ���������� (4,36v)
#define SET_IINLIM_1500mA 0x05U//���������� ������� ��� (1,5a)

//������������ �������� 01
#define POWER_ON_CONFIGURATION_REG 0x01U//�������� ����������� �������
#define POWER_ON_CONFIGURATION_RESET 0x80U
#define I2C_WATCHDOG_TIMER_RESET 0x40U
#define OTG_CONFIG 0x20U
#define CHG_CONFIG 0x10U
#define SET_SYS_MIN 0x05U// minimum system voltage limit (3.5v)
#define SET_BOOST_LIM 0x01U//1.5a - ���������� ���, ������������ �� OTG

//������������ �������� 02
#define CHARGE_CURRENT_CONTROL_REG 0x02U//�������� �������� ���
#define SET_ICHG_2000mA 0x60U//��� ������� ������� (2a �� �������� ������� 0x60), (3� � ������ 0xA0)
#define SET_ICHG_3000mA 0xA0//��� ������� ������� (2a �� �������� ������� 0x60), (3� � ������ 0xA0)
#define SET_BCOLD 0x02U// Set Boost Mode temperature monitor: threshold voltage to disable boost mode 0 � Vbcold0 (Typ. 76% of REGN or -10�Cw/ 103AT thermistor
#define SET_FORCE_20PCT 0x01U
/*
	0 � ICHG as Fast Charge Current
	(REG02[7:2]) and IPRECH as PreCharge Current (REG03[7:4])
	programmed
	1 � ICHG as 20% Fast Charge Current
	(REG02[7:2]) and IPRECH as 50% PreCharge Current (REG03[7:4])
	programmed
*/

//������������ �������� 03
#define PRE_CHARGE_TERMINATION_CURRENT_CONTROL_REG 0x03U//�������� ������� ���� ����������
#define SET_PRE_CHARGE_CURRENT_LIMIT_128mA 0x00U//��� ���������� (128��)
#define SET_PRE_CHARGE_CURRENT_LIMIT_256mA 0x10U//��� ���������� 256mA
#define SET_PRE_CHARGE_CURRENT_LIMIT_512mA 0x30U//��� ���������� 512mA
#define SET_PRE_CHARGE_CURRENT_LIMIT_1024mA 0x70U//��� ���������� 1024mA
#define SET_PRE_CHARGE_CURRENT_LIMIT_2048mA 0xF0U//��� ���������� 2048mA
//------------------------------------------------------------------------
#define SET_PRE_CHARGE_TERMINATION_LIMIT_128mA 0x00U//��� ���������� ������ 128 �� 
#define SET_PRE_CHARGE_TERMINATION_LIMIT_256mA 0x01U//��� ���������� ������ 256 �� 
#define SET_PRE_CHARGE_TERMINATION_LIMIT_512mA 0x03U//��� ���������� ������ 512 �� 
#define SET_PRE_CHARGE_TERMINATION_LIMIT_1024mA 0x07U//��� ���������� ������ 1024 �� 
#define SET_PRE_CHARGE_TERMINATION_LIMIT_1536mA 0x0BU//��� ���������� ������ 1536 �� 
#define SET_PRE_CHARGE_TERMINATION_LIMIT_2048mA 0x0F//��� ���������� ������ 2048 �� 

//������������ �������� 04
#define CHARGE_VOLTAGE_CONTROL_REG 0x04U //�������� ���������� ������, ����� �������� �� ���������� � ������� �������
#define SET_VREG_4016mV 0x80U //�������� ���������� ������
#define SET_VREG_4208mV 0xB0U //�������� ���������� ������
#define SET_BATLOWV_2800mV 0x00U//������� �� ���������� � ������� ������� 
#define SET_BATLOWV_3000mV 0x02U//������� �� ���������� � ������� ������� 
#define SET_VRECHG_100mV 0x00U //����� ���������� �������(0-100mV(default), 1-300mV) �� ��������� �������� ���� �������� VREG
#define SET_VRECHG_300mV 0x01U //����� ���������� �������(0-100mV(default), 1-300mV) �� ��������� �������� ���� �������� VREG

//������������ �������� 05
#define CHARGE_TERMINATION_TIMER_CONTROL_REG 0x05U//�������� ������ ���������� �������, i2c ������
#define EN_TERM 0x80U
#define SET_I2C_WATCHDOG_DISABLE 0x00U
#define SET_I2C_WATCHDOG_40sec 0x10U
#define SET_I2C_WATCHDOG_80sec 0x20U
#define SET_I2C_WATCHDOG_160sec 0x30U
#define EN_TIMER 0x08U//�������� ������ ������������ �������
//��������� ������� ���������� �������
#define SET_CHG_TIMER_5hrs 0x00U
#define SET_CHG_TIMER_8hrs 0x02U
#define SET_CHG_TIMER_12hrs 0x04U//�������� ������ ���������� ������� (12 ����� default)
#define SET_CHG_TIMER_20hrs 0x06U 

//������������ �������� 06
#define BOOST_VOLTAGE_THERMAL_REGULATION_CONTROL_REG 0x06U//�������� boost voltage, boost mode temperature monitor, ����� ������������ �� �����������
//BOOSTV range 4.55v-5.51v
#define SET_BOOSTV_4550mV 0x00U//�������� boost voltage
#define SET_BOOSTV_4998mV 0x70U//default
#define SET_BOOSTV_5510mV 0xF0U
//boost mode temperature monitor
#define SET_BHOOT_33pcnt_OF_REGN_OR_55degC 0x00U //Voltage to disable boost mode 00 � Vbhot1 (33% of REGN or 55�Cw/ 103AT thermistor)
#define SET_BHOOT_36pcnt_OF_REGN_OR_60degC 0x04U
#define SET_BHOOT_30pcnt_OF_REGN_OR_65degC 0x08U
#define SET_BHOOT_DISABLE 0x0CU
//thermal regulation treshold
#define SET_TREG_60deg 0x00U
#define SET_TREG_80deg 0x01U
#define SET_TREG_100deg 0x02U
#define SET_TREG_120deg 0x3U //default

//������������ �������� 07
#define MISC_OPERATION_CONTROL_REG 0x07U//

//-------------------�������� ���� ������ ��� ������-----------------------
#define SYSTEM_STATUS_REG 0x08U
//---4,5 bit
#define CHRG_STAT_NOT_CHARGING 0x00U
#define CHRG_STAT_PRE_CHARGE 0x10U
#define CHRG_STAT_FAST_CHARGING 0x20U
#define CHRG_STAT_CHARGE_TERMINATION_DONE 0x30U
//----------
#define PG_STAT_GOOD 0x04U
#define THERM_STAT_NORMAL 0x00U
#define THERM_STAT_REGULATION 0x02U
#define VSYS_STAT_REGULATION 0x01U //bat<vsysmin

#define NEW_FAULT_REG 0x09U
#define WATCHDOG_FAULT 0x80U
#define CHRG_FAULT_INPUT_FAULT 0x10U
#define CHRG_FAULT_THERMAL_SHUTDOWN 0x20U
#define CHRG_FAULT_CHARGE_TIMER_EXPIRATION 0x30U
#define BAT_FAULT 0x08U
#define NTC_FAULT_1 0x02U//����������� ���� ������ (�������)
#define NTC_FAULT 0x01U//����������� ���� ������ (�����)

//��������� ������� 24� �� ���� �������
//typedef enum 
//{
//	off,
//	on,
//}check24v_stat_type_def;



uint8_t check24v(void);

void bq2496_ini(void);


