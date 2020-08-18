#include <stdio.h>
#include "thread_funcs.h"


static pthread_t _threads[THREADS_MAX];
static unsigned int _numthreads = 0;


pthread_mutex_t msg_interlock;
pthread_cond_t msg_ready;
pthread_cond_t msg_sent;


static int
_threads_init_internal_stuff()
{
    pthread_mutexattr_t mattr;
    pthread_condattr_t cattr;
    int res = 0;

    pthread_mutexattr_init(&mattr);
    pthread_condattr_init(&cattr);

    if(0 != pthread_mutex_init(&msg_interlock, &mattr)) {
        perror("pthread_mutex_init()");
        res = -1;
    }

    if(0 != pthread_cond_init(&msg_ready, &cattr)) {
        perror("pthread_cond_init()");
        res = -2;
    }

    if(0 != pthread_cond_init(&msg_sent, &cattr)) {
        perror("pthread_cond_init()");
        res = -3;
    }

    pthread_mutexattr_destroy(&mattr);
    pthread_condattr_destroy(&cattr);

    return res;
}


int
threads_start(options_t *opts, thread_main_fn *threads, unsigned int nthreads)
{
    pthread_attr_t thread_attr;
    pthread_t thd;
    unsigned int i;

    if(0 == nthreads)
        return -1;

    pthread_attr_init(&thread_attr);
    for(i=0; i<nthreads; ++i) {
        if(NULL == threads[i]) {
            _threads[i] = 0;
            continue;
        }
        
        if(0 != pthread_create(&thd, &thread_attr, threads[i], opts)) {
            perror("pthread_create()");
            return -2;
        }
        printf("pthread_create(0x%08lx)=0x%08x\n", (unsigned long)threads[i], (unsigned int)thd);
        _threads[i] = thd;
    }

    if(0 != _threads_init_internal_stuff())
        return -3;

    _numthreads = nthreads;
    return 0;
}


void
threads_wait_complete()
{
    unsigned int i;

    for(i=0; i<_numthreads; ++i) {
        if(0 != _threads[i]) {
            if(0 != pthread_join(_threads[i], NULL)) {
                perror("pthread_join()");
                break;
            }
        }
    }
}
