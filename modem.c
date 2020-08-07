#include <termios.h>
#include <strings.h>
#include <sys/select.h>

#include "utils.h"
#include "modem.h"
#include "uart.h"

#include "nwy_uart.h"
#include "nwy_error.h"
#include "nwy_common.h"
#include "nwy_at.h"


int
modem_init(char *device, u_int32_t baudrate)
{
    int fd;

    printf("Init: MODEM\nOpening device %s\nBaudrate %u\n", device, baudrate);
    //fd = nwy_uart_open(device, baudrate, FC_NONE);

    fd = open(device, O_RDWR);
    if(fd < 0) {
        printf("nwy_uart_open() failed\n");
    }

    return fd;
}


static stage_fn
check_answer(options_t *opts, iobuf_t *answ, iobuf_t *reply, stage_fn fn)
{
    return (stage_fn)fn(opts, answ, reply);
}


static void *
stage_connect(options_t *opts, iobuf_t *answ, iobuf_t *reply)
{
    if(IS_OK(answ))
        return NULL;

    reply->len = sprintf((char *)reply->buf, "AT*99#\r\n");
    return stage_connect;
}


static void *
stage_set_credentials(options_t *opts, iobuf_t *answ, iobuf_t *reply)
{
    if(IS_OK(answ))
        return stage_connect;

    reply->len = sprintf((char *)reply->buf, "AT+XGAUTH=%d,%d,\"%s\",\"%s\"\r\n",
                         1,         // cid
                         1,         // 0=NONE, 1=PAP, 2=CHAP
                         "beeline", // login
                         "beeline"  // password
    );
    return stage_set_credentials;
}


static void *
stage_set_pdp_context(options_t *opts, iobuf_t *answ, iobuf_t *reply)
{
    if(IS_OK(answ))
        return stage_set_credentials;

    if(0 == strncasecmp((char *)answ->buf, "ERROR", 5))
        return NULL;

    reply->len = sprintf((char *)reply->buf, "AT+CGDCONT=%d,\"%s\",\"%s\",\"%s\",%d,%d\r\n",
                         1,                 // cid
                         "PPP",             // PDP type
                         "Beeline",         // APN
                         "217.118.87.98",   // PDP address: internet.beeline.ru
                         0,
                         0
    );

    return stage_set_pdp_context;
}


static void *
stage_set_baudrate(options_t *opts, iobuf_t *answ, iobuf_t *reply)
{
    if(IS_OK(answ))
        return stage_set_pdp_context;
    reply->len = sprintf((char *)reply->buf, "AT+IFR=%u\r\n", opts->modem_baud_rate);
    return stage_set_baudrate;
}


static void *
stage_send_pin(options_t *opts, iobuf_t *answ, iobuf_t *reply)
{
    if(IS_OK(answ))
        return stage_set_baudrate;

    reply->len = sprintf(reply->buf, "AT+CPIN=\"%s\"\r\n", opts->pin);
    return stage_send_pin;
}


static void *
stage_need_pin(options_t *opts, iobuf_t *answ, iobuf_t *reply)
{
    reply->len = 0;
    if(0 == strncasecmp((char *)answ->buf, "+CPIN: SIM PIN", 14))
        return stage_send_pin;
    return IS_OK(answ)? stage_set_baudrate: NULL;
}


static void *
stage_main(options_t *opts, iobuf_t *answ, iobuf_t *reply)
{
    (void)opts;

    if(IS_OK(answ)) {
        reply->len = sprintf((char *)&reply->buf, "AT+CPIN?");
        return stage_need_pin;
    }

    if('A' == answ->buf[0] && 'T' == answ->buf[1])
        return stage_need_pin;

    return NULL;
}


static void
hexdump(iobuf_t *buf)
{
    static char hexdigits[16] = "0123456789ABCDEF"; // lowercase maybe?
    char temp[1024];
    uint8_t c;
    u_int32_t i, n;

    for(i=0, n=0; i<buf->len; ++i) {
        c = buf->buf[i];
        temp[n++] = hexdigits[(c>>4) & 0x0F];
        temp[n++] = hexdigits[c & 0x0F];
        temp[n++] = ' ';
    }

    temp[n] = 0;
    printf(temp);
}


void *
modem_thread_main(void *arg)
{
    options_t *opts = (options_t *)arg;
    fd_set rfds, wfds;
    int fd, res;
    char *ch;
    iobuf_t inbuf, outbuf;
    stage_fn fn = stage_main;

    printf("Modem thread start\n", opts->modem_tty);

    fd = opts->modem_fd;

    inbuf.len = 0;
    outbuf.len = 0;

    /* Initiate modem setup sequence
     */
    strcpy((char *)outbuf.buf, "AT\r\n");
    outbuf.len = 4;

    for(;;) {
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_SET(fd, &rfds);
        if(outbuf.len > 0)
            FD_SET(fd, &wfds); // Do not check write access when there is no data to send

        res = select(fd+1, &rfds, &wfds, NULL, NULL);
        if(res < 0) {
            if(EINTR == errno)
                continue;
            break;
        }

        if(FD_ISSET(fd, &rfds)) {
            /* Read answer */
            //res = nwy_uart_read(fd, inbuf.buf+inbuf.len, UART_BUF_SIZE-inbuf.len);
            res = read(fd, inbuf.buf+inbuf.len, UART_BUF_SIZE-inbuf.len);
            if(res > 0) {
                inbuf.len += res;
                inbuf.buf[inbuf.len] = 0;

                ch = strchr((char*)inbuf.buf, '\r');
                if(NULL != ch && '\n' == ch[1]) {
                    /* We received full answer from neoway */
                    ch[2] = 0;
                    printf("(%d) %s", inbuf.len, (char *)inbuf.buf);
                    hexdump(&inbuf);
                    printf("\n");
                    fn = check_answer(opts, &inbuf, &outbuf, fn);
                    if(NULL == fn)
                        break; // Something went wrong!
                    inbuf.len = 0;
                }
            }
        }

        if(FD_ISSET(fd, &wfds)) {
            /* Send command */
            if(outbuf.len > 0) {
                //res = nwy_uart_write(fd, outbuf.buf, outbuf.len);
                res = write(fd, outbuf.buf, outbuf.len);
                outbuf.buf[outbuf.len] = 0;
                printf((char *)outbuf.buf);
                outbuf.len = 0;
            }
        }
    }

    return NULL;
}
