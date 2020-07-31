#ifndef MODEM_H
#define MODEM_H


#include <stdint.h>


#define UART_BUF_SIZE 1024


typedef struct _iobuf {
    int len;
    uint8_t buf[UART_BUF_SIZE];
} iobuf_t;


typedef void *(*stage_fn)(options_t *, iobuf_t *, iobuf_t *);


#define IS_OK(x) (('O'==(x)->buf[0])&&('K'==(x)->buf[1]))


#ifdef __cplusplus
extern "C" {
#endif
extern int modem_init(char *, u_int32_t);
extern void *modem_thread_main(void *);
#ifdef __cplusplus
}
#endif


#endif // MODEM_H
