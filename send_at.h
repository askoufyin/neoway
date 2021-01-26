#ifndef SENDAT_H
#define SENDAT_H


#include <stdint.h>
#include "utils.h"


// #ifdef __cplusplus
// extern "C" {
// #endif
void send_at_cmd(char* at_com, void* web_opts);     //Отправка АТ команды
void At_init(void* web_opts);                       //Инициализация для работы с AT

static void at_free(char** p);
// #ifdef __cplusplus
// }
// #endif


#endif // SENDAT_H
