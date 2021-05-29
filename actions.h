#ifndef __ACTIONS_H_INCLUDED__
#define __ACTIONS_H_INCLUDED__


#include "utils.h"


#ifdef __cplusplus
extern "C" {
#endif
char* arg_strdup(const char*);
actionarg_t* arg_new(const char*);
char* argval(options_t*, const char*);
actionarg_t* arg_find(options_t*, const char*, int);
void actions_init();
void which_action(options_t*, const char*);
#ifdef __cplusplus
}
#endif


#endif
