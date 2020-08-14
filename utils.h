#ifndef UTILS_H
#define UTILS_H


#include <stdint.h>
#include <netinet/in.h>
#include <pthread.h>

#include "lib/expat.h"


typedef int pid_t;


#define SOCKS_MAX               16
#define XML_VARIABLE_MAX        32
#define ACTION_NAME_MAX         32
#define PHONE_NUMBER_MAX        32
#define SMS_TEXT_MAX            512 // Bytes, not chars!
#define MAX_MESSAGE_LENGTH      8192


enum xml_element {
    XML_NONE = 0,
    XML_GET,
    XML_UUID,
    XML_BODY,
    XML_NAME,
    XML_VALUE,
    XML_ELEMENT_MAX
};
typedef struct _neoway_sms_data
{
    char phone[100][12];   //100 - максимальное количество смс в каждом приоритете
    char text[100][500];   //
    int j;              //Счетчик количества смс каждого приоритета
} neoway_sms_data_t;

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
    char *pid_file;                 // Name of the Process ID file
    char *kud_address;              // IP address of the KUD device
    char uuid[32];                  // Unique ID of the device for the UPVS protocol. Generated based on the device MAC address
    char r_uuid[32];                // UUID:service ID pair, received from the UPVS master
    uint32_t baud_rate;             // UART baud rate
    char *uart_tty;                 // Name of the UART TTY device
    uint32_t modem_baud_rate;       // Modem baud rate
    char *modem_tty;                // Name of the modem TTY device
    int go_daemon;                  // 0 = Run in foreground, any other value - become a daemon on startup
    int udp_broadcast;
    int tcp_sock;
    int broadcast_period;           // Announce broadcast period (sec)
    char *broadcast_addr;
    int debug_print;                //
    int gps_enabled;
    int gprs_enabled;
    struct sockaddr_in baddr;
    /* Фсякая бяка для УПВС */
    int level;                      // Nesting level of the current XML tag
    enum xml_element elem;
    enum xml_cmd xml_cmd;
    enum xml_action action;
    char xml_variable[XML_VARIABLE_MAX];
    char pin[4];
    actionhandler_fn xml_action;
    char phone_number[PHONE_NUMBER_MAX];
    char sms_text[SMS_TEXT_MAX];
    /* -- */
    pthread_mutex_t mutex;
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

