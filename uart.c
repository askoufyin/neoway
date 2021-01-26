#include "uart.h"
#include "modem.h"
#include "thread_funcs.h"

#include <strings.h>
#include <sys/select.h>
#include <termios.h>

#include "nwy_loc.h"
#include "nwy_uart.h"
#include "nwy_error.h"
#include "nwy_common.h"
#include "nwy_sim.h"


/* TODO: Make this variables static, add functions to manage data in the buffer
 */
extern char _sendbuf[MAX_MESSAGE_LENGTH+1];
extern size_t _sendbuflen;


int
uart_init(char *devname, uint32_t baudrate)
{
    int fd;

    printf("Init: UART\n");
    printf("Opening device %s\nBaudrate %u\n", devname, baudrate);
    fd = nwy_uart_open(devname, baudrate, FC_NONE);
    if(-1 == fd)
        perror("nwy_uart_open()");

    return fd;
}


static unsigned char
crc8(const char *data, size_t len)
{
    size_t i, j;
    unsigned char crc = 0xFF;

    for(i = 0; i < len; i++) {
        crc ^= data[i];
#if 0
        for(j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (unsigned char)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
#endif
    }

    return crc;
}


static const char _gps_prefix[] = "$MYGPSPOS: ";
#define _gps_prefix_len sizeof(_gps_prefix)


static int
process_command(options_t *opts, char *buffer) {
    static int turn = 1;
    int reset_mileage = 0;
    int res, len;
    unsigned char crc;
    char *reply, *status;
    char power_src[100];

    for(;'\0' != *buffer; ++buffer) {
        if(*buffer != ' ' && *buffer != '\t' && *buffer != '\n')
            break;
    }

    if('\0' == *buffer)
        return 0;

    if(0 == strncasecmp("INFO", buffer, 4)) {
        memset(power_src, 0, sizeof(power_src));
        pthread_mutex_lock(&opts->mutex);
        sscanf(buffer+5, "%f,%f,%s", &opts->total_mileage, &opts->mileage,power_src);
        pthread_mutex_unlock(&opts->mutex);

        printf("Total mileage: %f, mileage since last reset: %f\n", opts->total_mileage, opts->mileage);

        if(0 == strcasecmp("NO_BATTERY", power_src)) {
            opts->power_source = POWER_SOURCE_NORMAL;
            printf("Power source: NORMAL\n");
        } else {
            opts->power_source = POWER_SOURCE_BATTERY;
            printf("Power source: BATTERY\n");
        }

        _sendbuflen = snprintf(_sendbuf, MAX_MESSAGE_LENGTH, "$INFO,%s,%d,", NWY_SIM_READY==opts->sim_status? "V": "N", reset_mileage);
        crc = crc8(_sendbuf, _sendbuflen);
        _sendbuflen += sprintf(_sendbuf+_sendbuflen, "%02X\r\n", crc);
        _sendbuf[_sendbuflen] = 0;
        pthread_cond_signal(&msg_ready);
    }
    else
    if(0 == strncasecmp("AT", buffer, 2)) {
        len = strlen(buffer);
        reply = NULL;
        status = NULL;
        printf("wait mutex Leha!\n");
        pthread_mutex_lock(&opts->mutex_modem);
        printf("Sending %d bytes \"%s\" to modem\n", len, buffer);
        res = nwy_at_send_cmd(buffer, &reply, &status);
        printf("Reply from modem \"%s\" Status \"%s\" res=%d\n", (NULL==reply)? "": reply, status, res);

        if(NULL != reply && 0 == strncasecmp(_gps_prefix, reply, 11)) {
            pthread_mutex_lock(&opts->mutex);
            nmea_parse(reply+_gps_prefix_len, &opts->last_nmea_msg);
            pthread_mutex_unlock(&opts->mutex);
        }

        if(NULL != status)
        {
            _sendbuflen = snprintf(_sendbuf, MAX_MESSAGE_LENGTH, "%s\r\n", (NULL==reply)? status: reply);
            pthread_cond_signal(&msg_ready);
        }
        pthread_mutex_unlock(&opts->mutex_modem);
        free(reply);
        free(status);
    }

    return 0;
}


void *
uart_read_thread_main(void *arg)
{
    options_t *opts = (options_t *)arg;
    int res, len;
    fd_set rfds;
    size_t bufsize, cmdsize;
    char buffer[MAX_MESSAGE_LENGTH+1];
    char *crlf;
    int maxfd = MAX(opts->uart_fd, opts->modem_fd);
    struct timeval tm;

    printf("UART_READ thread start\n");

    bufsize = 0;
    for(;;) {
        FD_ZERO(&rfds);
        FD_SET(opts->uart_fd, &rfds);
        FD_SET(opts->modem_fd, &rfds);

        tm.tv_sec = 3;
        tm.tv_usec = 0;

        res = select(maxfd+1, &rfds, NULL, NULL, &tm); // blocking read
        if(-1 == res) {
            if(EINTR == errno)
                continue;
            perror("UART_READ select() failed");
            break;
        }

        if(0 == res) {
            printf(".\n");
            continue;
        }

        if(FD_ISSET(opts->uart_fd, &rfds)) {
            do {
                len = nwy_uart_read(opts->uart_fd, (unsigned char *)buffer + bufsize, sizeof(buffer)-bufsize);
                if(len > 0) {
                    bufsize += len;
                    crlf = strchr(buffer, '\r');
                    if(NULL != crlf && '\n' == crlf[1]) {
                        cmdsize = (crlf+1) - buffer;
                        *crlf = '\0';
                        printf("IN: \"%s\"\n", buffer);
                        process_command(opts, buffer);
                        memmove(buffer, crlf+1, bufsize - cmdsize);
                        bufsize -= cmdsize;
                    }
                }
            } while(len > 0);
        }
    }
}


void *
uart_write_thread_main(void *arg)
{
    options_t *opts = (options_t *)arg;
    int res, len;
    fd_set wfds;
//    struct timeval tm;

    printf("UART_WRITE thread start\n");

    for(;;) {
        pthread_cond_wait(&msg_ready, &msg_interlock);

        FD_ZERO(&wfds);
        FD_SET(opts->uart_fd, &wfds);

        res = select(opts->uart_fd+1, NULL, &wfds, NULL, NULL); // Blocking write
        if(res < 0) {
            if(EINTR == errno)
                continue;
            perror("select() failed");
            break;
        } else {
            if(FD_ISSET(opts->uart_fd, &wfds) && _sendbuflen > 0) {
                _sendbuf[_sendbuflen] = 0;
                printf("OUT: \"%s\"", _sendbuf);
                nwy_uart_write(opts->uart_fd, (const unsigned char *)_sendbuf, _sendbuflen);
                pthread_cond_signal(&msg_sent);
            }
        }
    }
}
