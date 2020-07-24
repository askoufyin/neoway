#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <pthread.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h> /* superset of previous */
#include <netdb.h>
#include <arpa/inet.h>

#include "nwy_loc.h"
#include "nwy_uart.h"
#include "nwy_error.h"
#include "nwy_common.h"

#include "config.h"
#include "main.h"
#include "nmea.h"
#include "at_commands.h"
#include "thread_funcs.h"
#include "modem.h"
#include "utils.h"


static location_rec_t _buffer[CIRCULAR_BUFFER_SIZE];
static pthread_mutex_t _bufmtx;
static u_int32_t _readptr;
static u_int32_t _writeptr;
static u_int32_t _count;

static int uart_fd, modem_fd;
static unsigned int _serial = 0;
static char _sendbuf[MAX_MESSAGE_LENGTH+1];
static size_t _sendbuflen = 0;


static void
process_msg_location(nwy_gps_location_t *loc)
{
    location_rec_t *rec;
    int i, len;
    unsigned char crc, *ptr;

    //printf("Long: %f, Lat: %f, Alt: %f, Speed: %f, Flags: 0x%08x\n", loc->longitude, loc->latitude, loc->altitude, loc->speed, loc->flags);

    /* Allocate record from circular buffer */
    rec = &_buffer[_writeptr];
    _readptr = _writeptr++;

    if(CIRCULAR_BUFFER_SIZE == _writeptr)
        _writeptr = 0; // wrap around buffer boundary

    if(_count < CIRCULAR_BUFFER_SIZE)
        ++_count;

    rec->timestamp = time(NULL);
    rec->latitude = loc->latitude;
    rec->longitude = loc->longitude;
    rec->altitude = loc->altitude;
    rec->speed = loc->speed;

    len = snprintf(_sendbuf, MAX_MESSAGE_LENGTH, "$PNGPS,%u,%ul,%f,%f,%f,%f",
                   _serial++, rec->timestamp,rec->latitude, rec->longitude, rec->altitude, rec->speed);

    _sendbuf[len] = '\0';

    for(i=1, ptr=(unsigned char *)_sendbuf, crc=0x00; i<len; ++i)
        crc ^= ptr[i];

    snprintf(_sendbuf+len, 5, "*%02x\r", crc);
    _sendbuf[len+5] = '\0';
    _sendbuflen = len+4; // for UART_WRITE thread.

    printf("%s", _sendbuf);

    pthread_cond_signal(&msg_ready); // wake up UART_WRITE thread
}


static void
nwy_loc_event_handler(nwy_loc_ind_t *msg)
{
    static nmea_msg_t nmsg;
    nmea_err_t err;
    int i;

    switch(msg->ind_type) {
        case NWY_LOC_LOCATION_INFO_IND_MSG:
            printf("NWY_LOC_LOCATION_INFO_IND_MSG\n");
            process_msg_location(&msg->ind_msg.gps_loc_msg);
            break;
#if 1
        case NWY_LOC_STATUS_INFO_IND_MSG:
            printf("NWY_LOC_STATUS_INFO_IND_MSG\n");
            break;

        case NWY_LOC_SV_INFO_IND_MSG:
            if(msg->ind_msg.sv_info_msg.size > 0) {
                printf("NWY_LOC_SV_INFO_IND_MSG (%d)\n\n", msg->ind_msg.sv_info_msg.size);
                for(i=0; i<msg->ind_msg.sv_info_msg.size; ++i) {
                    nwy_gps_sv_info_t *info = &msg->ind_msg.sv_info_msg.sv_list[i];
                    printf("\tAzimuth: %f\n", info->azimuth);
                    printf("\tElevation: %f\n", info->elevation);
                    printf("\tPRN: %d\n", info->prn);
                    printf("\tSNR: %f\n\n", info->snr);
                }
            }
            break;

        case NWY_LOC_NMEA_INFO_IND_MSG:
            err = nmea_parse(msg->ind_msg.nmea_msg.nmea, &nmsg);
            if(NMEA_ERR_OK == err) {
                if(NMEA_GSV == nmsg.type) {
                    if(nmsg.gsv.complete) {
                        //printf("%s: %d satellites\n", nmea_system_name(nmsg.type), nmsg.gsv.count);
                    }
                }
                else if(NMEA_GGA == nmsg.type) {

                } else {
                    //printf("NWY_LOC_NMEA_INFO_IND_MSG: %s", msg->ind_msg.nmea_msg.nmea);
                }
            } else {
                printf("nmea_parse(\"%s\") = %d\n", msg->ind_msg.nmea_msg.nmea, err);
            }
            break;

        case NWY_LOC_CAPABILITIES_INFO_IND_MSG:
            printf("NWY_LOC_CAPABILITIES_INFO_IND_MSG\n");
            break;

        case NWY_LOC_UTC_TIME_REQ_IND_MSG:
            printf("NWY_LOC_UTC_TIME_REQ_IND_MSG\n");
            break;

        case NWY_LOC_XTRA_DATA_REQ_IND_MSG:
            printf("NWY_LOC_XTRA_DATA_REQ_IND_MSG\n");
            break;

        case NWY_LOC_AGPS_STATUS_IND_MSG:
            printf("NWY_LOC_AGPS_STATUS_IND_MSG\n");
            break;

        case NWY_LOC_NI_NOTIFICATION_IND_MSG:
            printf("NWY_LOC_NI_NOTIFICATION_IND_MSG\n");
            break;

        case NWY_LOC_XTRA_REPORT_SERVER_IND_MSG:
            printf("NWY_LOC_XTRA_REPORT_SERVER_IND_MSG\n");
            break;
#endif
        default:
            break;
    }
}


int
agps_init()
{
    nwy_loc_position_mode_info_type_t pos_mode;
    int result;

    printf("Init: AGPS (v6)\n");

    nwy_loc_add_event_handler(nwy_loc_event_handler);

    result = nwy_loc_set_indication_mask(
#if 1
                NWY_LOC_INFO_IND_MASK
                | NWY_LOC_STATUS_INFO_IND_MASK
                | NWY_LOC_SV_INFO_IND_MASK
                | NWY_LOC_NMEA_INFO_IND_MASK
                | NWY_LOC_CAP_INFO_IND_MASK
                | NWY_LOC_UTC_REQ_IND_MASK
#else
                0x1FF /* All bits */
#endif
        );

    if (0 != result) {
        printf("nwy_loc_set_indication_mask() failed, error code = %d\n", result);
        return -1;
    }

    pos_mode.min_interval = 1000;
    pos_mode.mode = NWY_LOC_POS_MODE_AUTO; //STANDALONE;
    pos_mode.recurrence = NWY_LOC_POS_RECURRENCE_SINGLE; //PERIODIC;

    return nwy_loc_set_position_mode(pos_mode); //
}


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


int
network_init(options_t *opts)
{
    unsigned char mac[6];
    int sock, res, bcast_enable;
    struct hostent *hent;
    struct sockaddr_in kud;
    unsigned short broadcast_port = 19000;

    printf("Init: Network\n");
    res = get_UUID(mac);
    if(res < 0) {
        printf("get_UUID() failed. code=%d\n", res);
        return res;
    }

    sprintf(opts->uuid, "NEOWAY_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("UUID %s\n", opts->uuid);

    /* Create broadcast socket
     */
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock < 0) {
        perror("socket(UDP)");
        return -1;
    }

    opts->udp_broadcast = sock;

    bcast_enable = 1;
    res = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bcast_enable, sizeof(bcast_enable));
    if(res < 0) {
        perror("setsockopt()");
        return -1;
    }

    hent = gethostbyname(opts->broadcast_addr);
    if(NULL == hent) {
        printf("Cannot resolve host name %s\n", opts->broadcast_addr);
        return -2;
    }

    memset(&opts->baddr, 0, sizeof(opts->baddr));
    opts->baddr.sin_family = AF_INET;
    opts->baddr.sin_addr.s_addr = *((in_addr_t *)hent->h_addr_list[0]);
    opts->baddr.sin_port = htons(broadcast_port);

    printf("Broadcast address is %s:%u\n", inet_ntoa(opts->baddr.sin_addr), broadcast_port);

    opts->tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(opts->tcp_sock < 0) {
        perror("socket(TCP)");
        return -1;
    }

    hent = gethostbyname(opts->kud_address);
    if(NULL == hent) {
        printf("Cannot resolve host name %s\n", opts->broadcast_addr);
        return -2;
    }

    memset(&kud, 0, sizeof(kud));
    kud.sin_family = AF_INET;
    kud.sin_addr.s_addr = *((in_addr_t *)hent->h_addr_list[0]);
    kud.sin_port = htons(broadcast_port);

    printf("Gateway address is %s:%u\n", inet_ntoa(kud.sin_addr), broadcast_port);

    res = bind(opts->tcp_sock, (const struct sockaddr *)&kud, sizeof(kud));
    if(res < 0) {
        perror("bind()");
        return -2;
    }

    res = listen(opts->tcp_sock, 10); //connect(opts->tcp_sock, (struct sockaddr *)&kud, sizeof(kud));
    if(res < 0) {
        perror("listen()");
        return -3;
    }

    return 0;
}


static void *
agps_thread_main(void *arg)
{
    printf("AGPS thread start\n");
    nwy_loc_location_type_t loc_info;

    for(;;) {
        int result = nwy_loc_get_curr_location(&loc_info, 10); // sec or ms? assuming sec
        if(0 != result) {
            if(-703 == result) {
                printf("nwy_loc_get_curr_location() timeout\n");
            } else {
                printf("-----\nnwy_loc_get_curr_location() failed, returned value was %d\n", result);
                printf("Trying to recover\n-----\n");
            }
        }
    }
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


static void *
uart_read_thread_main(void *arg)
{
    options_t *opts = (options_t *)arg;
    int res, len;
    fd_set rfds;
    size_t bufsize, cmdsize;
    char buffer[MAX_MESSAGE_LENGTH+1];
    char *crlf;
    int maxfd;

    printf("UART_READ thread start\n");

//
    bufsize = 0;
    for(;;) {
        FD_ZERO(&rfds);
        FD_SET(uart_fd, &rfds);
        FD_SET(modem_fd, &rfds);

        res = select(uart_fd+1, &rfds, NULL, NULL, NULL); // blocking read
        if(-1 == res) {
            if(EINTR == errno)
                continue;
            perror("UART_READ select() failed");
            break;
        }

        if(FD_ISSET(uart_fd, &rfds)) {
            //memset(buffer, 0, sizeof(buffer));
            do {
                len = nwy_uart_read(uart_fd, (unsigned char *)buffer + bufsize, sizeof(buffer)-bufsize);
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


static void *
uart_write_thread_main(void *arg)
{
    int res, len;
    fd_set wfds;
//    struct timeval tm;

    (void)arg;

    printf("UART_WRITE thread start\n");
    for(;;) {
        pthread_cond_wait(&msg_ready, &msg_interlock);

        FD_ZERO(&wfds);
        FD_SET(uart_fd, &wfds);

        res = select(uart_fd+1, NULL, &wfds, NULL, NULL); // Blocking write
        if(-1 == res) {
            if(EINTR == errno)
                continue;
            perror("select() failed");
            break;
        } else {
            if(FD_ISSET(uart_fd, &wfds)) {
                nwy_uart_write(uart_fd, (const unsigned char *)_sendbuf, _sendbuflen);
                pthread_cond_signal(&msg_sent);
            }
        }
    }
}


static void
broadcast(options_t *opts)
{
    int len, res;
    char buf[512];

    printf("Broadcasting announce\n");

    len = snprintf(buf, sizeof(buf),
             "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
             "<info>\n"
             "<URL>%s:%d</URL>\n"
             "<UUID>%s</UUID>\n"
             "<TTL>%d</TTL>\n"
             "</info>\r\n", GATEWAY_ADDRESS, 19000, opts->uuid, opts->broadcast_period*2
             );

    res = sendto(opts->udp_broadcast, buf, len, 0, (const struct sockaddr *)&opts->baddr, sizeof(opts->baddr));
    if(res < 0) {
        perror("sendto()");
    }
}


static void
receive_command(options_t *opts, fd_set *fds)
{
    char buf[8192];
    int res;
    struct sockaddr_in sin;
    socklen_t sinsize = sizeof(sin);

#if 0
    if(FD_ISSET(opts->tcp_sock, fds)) {
        res = recv(opts->tcp_sock, buf, sizeof(buf), 0);
        if(res < 0)
            perror("recv()");
        else if(0 == res) {
            printf("Other side closed connection\n");
        }
        printf("Received %d bytes\n", res);
    }
#endif

    if(FD_ISSET(opts->udp_broadcast, fds)) {
        sin.sin_family = AF_INET;
        sin.sin_addr.s_addr = opts->baddr.sin_addr.s_addr;
        sin.sin_port = htons(19000);
        res = recvfrom(opts->udp_broadcast, buf, sizeof(buf), 0, (struct sockaddr *)&sin, &sinsize);
        if(res < 0)
            perror("recvfrom()");

        printf("Datagram received (%d bytes) from %s\n", res, inet_ntoa(sin.sin_addr));
    }

    if(res >= 0) {
        buf[res] = 0;
        printf("%s\n", buf);
    }
}


static void
process_get(options_t *opts)
{
    char *p = strchr(opts->r_uuid, ':');

    if(NULL == p) {
        _sendbuflen = sprintf(_sendbuf,
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            "<root>\n"
                "<device>\n"
                    "<UUID>%s</UUID>\n"
                    "<deviceType></deviceType>\n"
                    "<serialNumber>000000000000</serialNumber>\n"
                    "<modelName>Neoway N720</modelName>\n"
                    "<friendlyName>Neoway</friendlyName>\n"
                    "<company>KSC Elcom</company>\n"
                    "<country>Russia</country>\n"
                    "<version>0.1</version>\n"
                    "<serviceList>\n"
                        "<service>\n"
                            "<serviceType>SMS gate</serviceType>\n"
                            "<serviceID>SMSG</serviceID>\n"
                        "</service>\n"
                    "</serviceList>\n"
                "</device>\n"
            "</root>\r\n",
            opts->uuid
        );
    } else {
        ++p;
        if(0 == strcasecmp(p, "SMSG")) {
            _sendbuflen = sprintf(_sendbuf,
                "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                "<scpd urn=\"%s\">\n"
                    "<actionList>\n"
                        "<action>\n"
                            "<name>Send SMS</name>\n"
                            "<argumentList>\n"
                                "<name>Text</name>\n"
                                "<direction>IN</direction>\n"
                            "</argumentList>\n"
                        "</action>\n"
                    "</actionList>\n"
                    "<serviceStateTable>\n"
                        "<stateVariable>\n"
                            "<name>text</name>\n"
                            "<dataType>string</dataType>\n"
                        "</stateVariable>\n"
                    "</serviceStateTable>\n"
                "</scpd>\r\n",
                opts->r_uuid
            );
        }
    }
}


static void XMLCALL
process_character_data(void *userdata, const XML_Char *text, int len)
{
    options_t *opts = (options_t *)userdata;

    switch(opts->elem) {
        case XML_UUID:
            strncpy(opts->r_uuid, text, len);
            opts->r_uuid[len] = 0;
            printf("UUID = %s\n", opts->r_uuid);
            break;
    }
}


static void XMLCALL
process_element_start(void *userdata, const XML_Char *name, const XML_Char **attrs)
{
    options_t *opts = (options_t *)userdata;

    printf("%s\n", name);

    if(0 == strcasecmp(name, "get")) {
        printf("XML_GET\n");
        opts->elem = XML_GET;
    }

    if(0 == strcasecmp(name, "UUID")) {
        opts->elem = XML_UUID;
        printf("XML_UUID\n");
    }

    ++opts->level;
}


static void XMLCALL
process_element_end(void *userdata, const XML_Char *name)
{
     options_t *opts = (options_t *)userdata;

    (void)name;

    if(--opts->level == 0) {
        printf("process_get()\n");
        process_get(opts);
    }
}


static void *
network_thread_main(void *arg)
{
    options_t *opts = (options_t *)arg;
    fd_set rfds, wfds;
    struct timeval tm;
    int res, maxsock;
    time_t last_ts, now;
    int i, sock;
    char buf[8192];
    XML_Parser parser;

    parser = XML_ParserCreate(NULL);

    printf("Network thread start\n");

    last_ts = time(NULL);
    for(;;) {
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);

        FD_SET(opts->udp_broadcast, &wfds);
        FD_SET(opts->tcp_sock, &rfds);

        maxsock = MAX(opts->udp_broadcast, opts->tcp_sock);

        tm.tv_sec = opts->broadcast_period;
        tm.tv_usec = 0;

        res = select(maxsock+1, &rfds, &wfds, NULL, &tm);
        now = time(NULL);
        if(res < 0) {
            if(EINTR == errno)
                continue;
        } else if(res == 0) { // timeout reached
            broadcast(opts);
            last_ts = now;
        } else {
            if(last_ts + opts->broadcast_period < now) { // timeout reached
                broadcast(opts);
                last_ts = now;
            }

            if(FD_ISSET(opts->tcp_sock, &rfds)) {
                struct sockaddr_in in_addr;
                socklen_t len = sizeof(in_addr);

                sock = accept(opts->tcp_sock, (struct sockaddr *)&in_addr, &len);
                printf("Accepted connection from %s:%u\n", inet_ntoa(in_addr.sin_addr), ntohs(in_addr.sin_port));

                res = recv(sock, buf, sizeof(buf), 0);
                if(res > 0) {
                    memset(_sendbuf, 0, sizeof(_sendbuf));

                    printf("%s", buf);

                    XML_SetUserData(parser, opts);
                    XML_SetStartElementHandler(parser, process_element_start);
                    XML_SetEndElementHandler(parser, process_element_end);
                    XML_SetCharacterDataHandler(parser, process_character_data);
                    XML_Parse(parser, buf, res, TRUE);

                    process_get(opts);

                    printf("%s", _sendbuf);
                    res = send(sock, _sendbuf, _sendbuflen, 0);

                    XML_ParserReset(parser, NULL);
                }

                shutdown(sock, SHUT_RDWR);
                close(sock);
            }

            if(FD_ISSET(opts->udp_broadcast, &rfds)) {
                receive_command(opts, &rfds);
            }
        }
    }

    return NULL;
}


static int
parse_command_line(options_t *opts, int argc, char *argv[])
{
    struct option _options[] = {
        { "daemonize",  no_argument,       &opts->go_daemon, 1 },   // 0
        { "pidfile",    required_argument, NULL, 0 },               // 1
        { "help",       no_argument,       NULL, 0 },               // 2
        { "config",     required_argument, NULL, 0 },               // 3
        { "kud",        required_argument, NULL, 0 },               // 4
        { "baudrate",   required_argument, NULL, 0 },               // 5
        { "uart",       required_argument, NULL, 0 },               // 6
        { "modem",      required_argument, NULL, 0 },               // 7
        { NULL, 0, NULL, 0 } // end
    };
    const char _shortopts[] = "dp:c:k:r:u:h";
    const char _shortopts_chars[] = "dpckrush"; // without special chars and in the exact same order as _options
    int opt, idx, argval;
    char *end;

    while((opt = getopt_long(argc, argv, _shortopts, _options, &idx)) >= 0) {
        if(idx < 0) {
            for(idx=0; '\0' != _shortopts_chars[idx]; ++idx)
                if(_shortopts_chars[idx] == opt)
                    break;

            if(idx < 0)
                return -1;
        }

        switch(idx) {
            case 0: // nodaemon
                opts->go_daemon = 0;
                break;

            case 1: // pidfile
                free(opts->pid_file);
                opts->pid_file = strdup(optarg);
                break;

            case 2: // help
                help(opts);
                return 1;

            case 3: // config
                // TODO
                break;

            case 4: // kud
                // TODO
                break;

            case 5: // baudrate
                argval = (int)strtol(optarg, &end, 10);
                if('\0' == *end) {
                    opts->baud_rate = atoi(optarg);
                } else {
                    fprintf(stderr, "Error: argument of --baudrate is not a valid number\n");
                }
                break;

            case 6: // uart
                free(opts->uart_tty);
                opts->uart_tty = strdup(optarg);
                break;

            case 7: // modem
                free(opts->modem_tty);
                opts->modem_tty = strdup(optarg);
                break;


            default:
                break;
        }
    }

    return 0;
}


int
app_init(options_t *opts)
{
    if(0 != network_init(opts)) {
        printf("Network init failed\n");
        exit(EXIT_FAILURE);
    }

    uart_fd = uart_init(opts->uart_tty, opts->baud_rate);
    if(-1 == uart_fd) {
        printf("UART init failed\n");
        exit(EXIT_FAILURE);
    }

    opts->uart_fd = uart_fd;

    modem_fd = modem_init(opts->modem_tty, opts->baud_rate);
    if(-1 == modem_fd) {
        printf("MODEM init failed\n");
        exit(EXIT_FAILURE);
    }

    opts->modem_fd = modem_fd;

    if(0 != agps_init()) {
        printf("AGPS init failed\n");
        //exit(EXIT_FAILURE);
    }

    return 0;
}


int
main(int argc, char *argv[])
{
    options_t opts;
    thread_main_fn threads[] = { /* agps_thread_main, */ uart_read_thread_main, uart_write_thread_main, network_thread_main, modem_thread_main };

    options_init(&opts);
    parse_command_line(&opts, argc, argv);

    if(0 != app_init(&opts)) {
        options_cleanup(&opts);
        return EXIT_FAILURE;
    }

    if(opts.go_daemon) {
        daemonize(&opts, 0);
    }

    if(0 == threads_start(&opts, threads, sizeof(threads)/sizeof(threads[0]))) {
        printf("Running\n");
        threads_wait_complete();
    } else {
        options_cleanup(&opts);
        return EXIT_FAILURE;
    }

    options_cleanup(&opts);
    return EXIT_SUCCESS;
}

