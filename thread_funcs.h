#ifndef THREAD_FUNCS_H
#define THREAD_FUNCS_H

#include <pthread.h>
#include "utils.h"


#define THREADS_MAX 8


typedef void *(*thread_main_fn)(void *);


#ifdef __cplusplus
extern "C" {
#endif
/* variables */
extern pthread_mutex_t msg_interlock;
extern pthread_cond_t msg_ready;
extern pthread_cond_t msg_sent;
/* functions */
extern int threads_start(options_t *, thread_main_fn *, unsigned int);
extern void threads_wait_complete();
#ifdef __cplusplus
}
#endif


#endif // THREAD_FUNCS_H
