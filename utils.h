#ifndef UTILS_H
#define UTILS_H


#include <stdint.h>
#include <netinet/in.h>

#include "lib/expat.h"


typedef int pid_t;


#define SOCKS_MAX 16
#define XML_VARIABLE_MAX    32
#define ACTION_NAME_MAX     32
#define PHONE_NUMBER_MAX    32
#define SMS_TEXT_MAX        512 // Bytes, not chars!


enum xml_element {
    XML_NONE = 0,
    XML_GET,
    XML_UUID,
    XML_BODY,
    XML_NAME,
    XML_VALUE,
    XML_ELEMENT_MAX
};


enum xml_cmd {
    XML_CMD_NONE = 0,
    XML_CMD_QUERY_STATE_VARIABLE,
    XML_CMD_ACTION,
    XML_CMD_ACTION_NAME,
    XML_CMD_ACTION_ARGS
};


enum xml_action {
    XML_ACTION_NONE = 0,
    XML_ACTION_SEND_SMS
};


struct _options;


typedef void (*actionhandler_fn)(struct _options *);


typedef struct _options {
    char *pid_file;
    char *kud_address;
    char uuid[32];
    char r_uuid[32];
    uint32_t baud_rate;
    char *uart_tty;
    uint32_t modem_baud_rate;
    char *modem_tty;
    int go_daemon;
    int udp_broadcast;
    int tcp_sock;
    int broadcast_period;
    char *broadcast_addr;
    struct sockaddr_in baddr;
    /* Фсякая бяка для УПВС */
    int level;
    enum xml_element elem;
    enum xml_cmd xml_cmd;
    enum xml_action action;
    char xml_variable[XML_VARIABLE_MAX];
    char pin[4];
    actionhandler_fn xml_action;
    char phone_number[PHONE_NUMBER_MAX];
    char sms_text[SMS_TEXT_MAX];
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
    char carrige_mileage[10];
    char last_mileage[10];
    char power_type[10];
    char up_time_string[20];
    char country_cod[10];
    char operator_cod[10];



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
