#ifndef UTILS_H
#define UTILS_H


#include <stdint.h>
#include <netinet/in.h>

#include "lib/expat.h"


typedef int pid_t;


#define SOCKS_MAX 16


enum xml_element {
    XML_NONE = 0,
    XML_GET,
    XML_UUID
};


typedef struct _options {
    char *pid_file;
    char *kud_address;
    char uuid[32];
    char r_uuid[32];
    uint32_t baud_rate;
    char *uart_tty;
    uint32_t modem_baud_rate;
    char *modem_tty;
    char pin[4];
    int go_daemon;
    int udp_broadcast;
    int tcp_sock;
    int broadcast_period;
    char *broadcast_addr;
    struct sockaddr_in baddr;
    int level;
    enum xml_element elem;
    /* -- */
    int modem_fd;
    int uart_fd;
} options_t;


#define MAX(x,y)    ((x)>=(y)? (x): (y))


#ifdef __cplusplus
extern "C" {
#endif
//extern options_t opts;
extern void options_init(options_t *);
extern void options_cleanup(options_t *);
extern int daemonize(options_t *, int);
extern int write_pid(options_t *, pid_t);
extern void help(options_t *);
extern int get_UUID(unsigned char *);
#ifdef __cplusplus
}
#endif


#endif // UTILS_H
