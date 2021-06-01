#ifndef CONFIG_H
#define CONFIG_H


#define APPNAME                 "neowayhelper"
#define MAXFD                   4096
#define WORKDIR                 "/data"
#define CONFDIR                 "/etc"
#define CONFIG_FILE             CONFDIR "/" APPNAME ".conf"
#define DEFAULT_UART_BAUD_RATE  115200
#define DEFAULT_MODEM_BAUD_RATE 115200
#define DEFAULT_UART_TTY        "/dev/ttyHSL0"
#define DEFAULT_MODEM_TTY       "/dev/smd8"
#define PID_FILE                "/var/run/" APPNAME ".pid"
#define UMASK                   (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH)
#define BROADCAST_ADDRESS       "10.7.255.255"
#define BROADCAST_PERIOD        5
//#define GATEWAY_ADDRESS         "10.7.6.1"
#define GATEWAY_ADDRESS         "10.7.254.23"
#define DEVICE_TYPE             "NEOWAY_"


#ifndef TRUE
#define TRUE 1
#endif


#ifndef FALSE
#define FALSE 0
#endif


#endif // CONFIG_H
