
/*
AT+GETMIL - получение общего пробега
AT+GETPMIL - получение промежуточного пробега
AT+SETMIL= - установить (перезаписать) пробег
AT+SETPMIL= - установить (перезаписать) промежуточный пробег
AD+GPSPOS - запрос геопозиции
AD+SHUTDOWN - выключить модуль
AD+POWER - включить модуль
AD+REBOOT - перезагрузить stm
*/

#include "main.h"

uint8_t check_cmd(void);
