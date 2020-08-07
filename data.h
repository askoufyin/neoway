#ifndef __DATA_H__
#define __DATA_H__


#include "utils.h"


#ifdef __cplusplus
extern "C" {
#endif
extern int data_thread_init(options_t *);
extern void establish_data_connection(options_t *);
#ifdef __cplusplus
}
#endif


#endif
