#ifndef UTILS_H
#define UTILS_H


#include <stdint.h>
#include <netinet/in.h>
#include <pthread.h>

#include "lib/expat.h"

#include "nmea.h"


typedef int pid_t;


#define SOCKS_MAX               16
#define XML_VARIABLE_MAX        32
#define ACTION_NAME_MAX         32
#define PHONE_NUMBER_MAX        32
#define SMS_TEXT_MAX            512 // Bytes, not chars!
#define MAX_MESSAGE_LENGTH      8192

#define MAX_PATH_LENGTH 256


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
    char phone[100][13];   //100 - максимальное количество смс в каждом приоритете
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


typedef enum {
    POWER_SOURCE_BATTERY,           // Питание от батареи
    POWER_SOURCE_NORMAL             // Питание от сети
} power_source_t;


typedef struct _options {
    char *pid_file;                 // Name of the Process ID file
    char *kud_address;              // IP address of the KUD device
    char uuid[32];                  // Unique ID of the device for the UPVS protocol. Generated based on the device MAC address
    char r_uuid[32];                // UUID:service ID pair, received from the UPVS master
    char *uart_tty;                 // Name of the UART TTY device
    int uart_baud_rate;             // UART baud rate
    int modem_baud_rate;            // Modem baud rate
    char *modem_tty;                // Name of the modem TTY device
    int go_daemon;                  // 0 = Run in foreground, any other value - become a daemon on startup
    int udp_broadcast;
    int tcp_sock;
    int broadcast_period;           // Announce broadcast period (sec)
    char *broadcast_addr;
    int debug_print;                //
    int sim_status;
    int gps_enabled;
    /* GPRS */
    int gprs_enabled;
    char *gprs_apn;                 // Access point name
    char *gprs_user;
    char *gprs_password;
    char *gprs_post_connect;
    struct sockaddr_in baddr;
    /* Синхронизация */
    pthread_mutex_t mutex;
    pthread_mutex_t mutex_modem;
    /* GPS-координаты */
    nmea_msg_t last_nmea_msg;
    float total_mileage;            // Полный пробег, км
    float mileage;                  // Пробег с последнего сброса, км
    int reset_mileage;              // 1 - нужно сбросить пробег, 0 - ненужно сбрасывать
    power_source_t power_source;    // Источник питания
    /* Фсякая бяка для УПВС */
    int level;                      // Nesting level of the current XML tag
    enum xml_element elem;
    enum xml_cmd xml_cmd;
    enum xml_action action;
    char xml_variable[XML_VARIABLE_MAX];
    char pin[5];                    // With trailing \0
    actionhandler_fn xml_action;
    char phone_number[PHONE_NUMBER_MAX];
    char sms_text[SMS_TEXT_MAX];
    /* -- */
    int modem_fd;
    int uart_fd;
    /* -web interface- */
    char *web_dir_i_path;
    char rssi[15];
    char threed_fix[10];
    char gps_cords[30];
    char reg_in_mesh[10];
    char mobile_data[10];
    char imsi[20];
    char imei[20];
    char up_time_string[20];
    char country_cod[20];
    char operator_cod[20];
    float lon;
    float lat;
    char lat_sign;
    char lon_sign;
    char valid_GPRMC;
    char sput_time[20];
    char gsm_ip_state[100];
    int num_sput_val;
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
//extern int write_pid(options_t *, pid_t);
extern void help(options_t *);
extern int get_UUID(unsigned char *);
#ifdef __cplusplus
}
#endif


#endif // UTILS_H
