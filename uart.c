#include "uart.h"
#include "modem.h"
#include "thread_funcs.h"

#include <sys/select.h>
#include <termios.h>

#include "nwy_loc.h"
#include "nwy_uart.h"
#include "nwy_error.h"
#include "nwy_common.h"


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


static int
process_command(char *buffer) {
    static int turn = 1;

    for(;'\0' != *buffer; ++buffer) {
        if(*buffer != ' ' && *buffer != '\t' && *buffer != '\n')
            break;
    }

    if('\0' == *buffer)
        return 0;

    if(0 == strcasecmp("RUN_SERVICE", buffer)) {
        printf("OK\n");
        _sendbuflen = snprintf(_sendbuf, MAX_MESSAGE_LENGTH, "OK (%d)\r", turn++);
        pthread_cond_signal(&msg_ready);
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

    printf("UART_READ thread start\n");

    bufsize = 0;
    for(;;) {
        FD_ZERO(&rfds);
        FD_SET(opts->uart_fd, &rfds);
        FD_SET(opts->modem_fd, &rfds);

        res = select(maxfd+1, &rfds, NULL, NULL, NULL); // blocking read
        if(-1 == res) {
            if(EINTR == errno)
                continue;
            perror("UART_READ select() failed");
            break;
        }

        if(FD_ISSET(opts->uart_fd, &rfds)) {
            //memset(buffer, 0, sizeof(buffer));
            do {
                len = nwy_uart_read(opts->uart_fd, (unsigned char *)buffer + bufsize, sizeof(buffer)-bufsize);
                if(len > 0) {
                    bufsize += len;
                    crlf = strchr(buffer, '\r');
                    if(NULL != crlf) {
                        cmdsize = (crlf+1) - buffer;
                        *crlf = '\0';
                        printf("%s\n", buffer);
                        process_command(buffer);
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
            if(FD_ISSET(opts->uart_fd, &wfds)) {
                nwy_uart_write(opts->uart_fd, (const unsigned char *)_sendbuf, _sendbuflen);
                pthread_cond_signal(&msg_sent);
            }
        }
    }
}
