#ifndef UART_H
#define UART_H


#include <stdint.h>
#include <pthread.h>
#include "utils.h"


#ifdef __cplusplus
extern "C" {
#endif
extern int uart_init(char *, uint32_t);
extern void *uart_read_thread_main(void *);
extern void *uart_write_thread_main(void *);
#ifdef __cplusplus
}
#endif


#endif // UART_H
