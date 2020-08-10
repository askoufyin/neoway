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
typedef struct _neoway_sms_data
{
    char phone[100][12];   //100 - максимальное количество смс в каждом приоритете
    char text[100][500];   //
    int j;              //Счетчик количества смс каждого приоритета
} neoway_sms_data_t;

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
    /* -web interface- */
    char rssi[5];
    char rsrq[10];        //unused
    char snr[10];         //unused
    char rxlen[10];       //unused
    char spn[10];         //unused
    char threed_fix[10];
    char gps_cords[30];
    char reg_in_mesh[10];
    char mobile_data[10];
    char imsi[20];
    char imei[20];
    char carrige_mileage[10]; //общий пробег
    char last_mileage[10];    //пробег с последнего старта? (предположительно старта неовэя)
    char power_type[10];      //Батарея или сеть
    char up_time_string[20];
    char country_cod[10];
    char operator_cod[10];
    /*-Work whith SMS-*/
    int sms_priority;
    neoway_sms_data_t queue[8],sended[8],deleted[8],recved[8];



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

