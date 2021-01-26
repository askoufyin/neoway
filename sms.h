#ifndef SMS_H
#define SMS_H


#include <stdint.h>
#include <pthread.h>
#include "utils.h"


#ifdef __cplusplus
extern "C" {
#endif
extern int Read_smsFrom_txt(const char* file, void* web_opts);
extern int Write_smsTo_txt(const char* file, void* web_opts);
#ifdef __cplusplus
}
#endif


#endif // SMS_H
