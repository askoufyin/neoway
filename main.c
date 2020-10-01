#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <pthread.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/ip.h> /* superset of previous */
#include <netdb.h>
#include <arpa/inet.h>
#include <stdbool.h> //
#include <signal.h> //

#include "nwy_loc.h"
#include "nwy_uart.h"
#include "nwy_error.h"
#include "nwy_common.h"
#include "nwy_gpio.h"   //для хартбита
#include "nwy_pm.h"     //для хз чего
#include "nwy_sms.h"    //для отправки смс
#include "nwy_sim.h"

#include "config.h"
#include "configfile.h"
#include "main.h"
#include "nmea.h"
#include "data.h"
#include "at_commands.h"
#include "thread_funcs.h"
#include "uart.h"
#include "modem.h"
#include "utils.h"
#include "watchdog.h"
#include "nwy_sim.h"

//#include "tcp_web.h"   пока не добавлено
#define WEBPOSTGETINCONSOLE  //убрать если отвлекает вывод пост гет запросов в консоли

static location_rec_t _buffer[CIRCULAR_BUFFER_SIZE];
static pthread_mutex_t _bufmtx;
static u_int32_t _readptr;
static u_int32_t _writeptr;
static u_int32_t _count;

static unsigned int _serial = 0;
char _sendbuf[MAX_MESSAGE_LENGTH + 1];
size_t _sendbuflen = 0;


static void
process_msg_location(nwy_gps_location_t* loc)
{
    location_rec_t* rec;
    int i, len;
    unsigned char crc, * ptr;

    //printf("Long: %f, Lat: %f, Alt: %f, Speed: %f, Flags: 0x%08x\n", loc->longitude, loc->latitude, loc->altitude, loc->speed, loc->flags);

    /* Allocate record from circular buffer */
    rec = &_buffer[_writeptr];
    _readptr = _writeptr++;

    if (CIRCULAR_BUFFER_SIZE == _writeptr)
        _writeptr = 0; // wrap around buffer boundary

    if (_count < CIRCULAR_BUFFER_SIZE)
        ++_count;

    rec->timestamp = time(NULL);
    rec->latitude = loc->latitude;
    rec->longitude = loc->longitude;
    rec->altitude = loc->altitude;
    rec->speed = loc->speed;

    len = snprintf(_sendbuf, MAX_MESSAGE_LENGTH, "$PNGPS,%u,%ul,%f,%f,%f,%f",
        _serial++, rec->timestamp, rec->latitude, rec->longitude, rec->altitude, rec->speed);

    _sendbuf[len] = '\0';

    for (i = 1, ptr = (unsigned char*)_sendbuf, crc = 0x00; i < len; ++i)
        crc ^= ptr[i];

    snprintf(_sendbuf + len, 5, "*%02x\r", crc);
    _sendbuf[len + 5] = '\0';
    _sendbuflen = len + 4; // for UART_WRITE thread.

    printf("%s", _sendbuf);

    pthread_cond_signal(&msg_ready); // wake up UART_WRITE thread
}


static void
nwy_loc_event_handler(nwy_loc_ind_t* msg)
{
    static nmea_msg_t nmsg;
    nmea_err_t err;
    int i;

    switch (msg->ind_type) {
    case NWY_LOC_LOCATION_INFO_IND_MSG:
        printf("NWY_LOC_LOCATION_INFO_IND_MSG\n");
        process_msg_location(&msg->ind_msg.gps_loc_msg);
        break;
#if 1
    case NWY_LOC_STATUS_INFO_IND_MSG:
        printf("NWY_LOC_STATUS_INFO_IND_MSG\n");
        break;

    case NWY_LOC_SV_INFO_IND_MSG:
        if (msg->ind_msg.sv_info_msg.size > 0) {
            printf("NWY_LOC_SV_INFO_IND_MSG (%d)\n\n", msg->ind_msg.sv_info_msg.size);
            for (i = 0; i < msg->ind_msg.sv_info_msg.size; ++i) {
                nwy_gps_sv_info_t* info = &msg->ind_msg.sv_info_msg.sv_list[i];
                printf("\tAzimuth: %f\n", info->azimuth);
                printf("\tElevation: %f\n", info->elevation);
                printf("\tPRN: %d\n", info->prn);
                printf("\tSNR: %f\n\n", info->snr);
            }
        }
        break;

    case NWY_LOC_NMEA_INFO_IND_MSG:
        err = nmea_parse(msg->ind_msg.nmea_msg.nmea, &nmsg);
        if (NMEA_ERR_OK == err) {
            if (NMEA_GSV == nmsg.type) {
                if (nmsg.gsv.complete) {
                    //printf("%s: %d satellites\n", nmea_system_name(nmsg.type), nmsg.gsv.count);
                }
            }
            else if (NMEA_GGA == nmsg.type) {

            }
            else {
                //printf("NWY_LOC_NMEA_INFO_IND_MSG: %s", msg->ind_msg.nmea_msg.nmea);
            }
        }
        else {
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
network_init(options_t* opts)
{
    unsigned char mac[6];
    int sock, res, bcast_enable;
    struct hostent* hent;
    struct sockaddr_in kud;
    unsigned short broadcast_port = 19001; // TODO: Move this constant to options_t!

    printf("Init: Network\n");
    res = get_UUID(mac);
    if (res < 0) {
        printf("get_UUID() failed. code=%d\n", res);
        return res;
    }

    sprintf(opts->uuid, "NEOWAY_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("UUID %s\n", opts->uuid);

    /* Create broadcast socket
     */
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("socket(UDP)");
        return -1;
    }

    opts->udp_broadcast = sock;

    bcast_enable = 1;
    res = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bcast_enable, sizeof(bcast_enable));
    if (res < 0) {
        perror("setsockopt()");
        return -1;
    }

    hent = gethostbyname(opts->broadcast_addr);
    if (NULL == hent) {
        printf("Cannot resolve host name %s\n", opts->broadcast_addr);
        return -2;
    }

    memset(&opts->baddr, 0, sizeof(opts->baddr));
    opts->baddr.sin_family = AF_INET;
    opts->baddr.sin_addr.s_addr = *((in_addr_t*)hent->h_addr_list[0]);
    opts->baddr.sin_port = htons(broadcast_port);

    printf("Broadcast address is %s:%u\n", inet_ntoa(opts->baddr.sin_addr), broadcast_port);

    opts->tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (opts->tcp_sock < 0) {
        perror("socket(TCP)");
        return -1;
    }

    hent = gethostbyname(opts->kud_address);
    if (NULL == hent) {
        printf("Cannot resolve host name %s\n", opts->broadcast_addr);
        return -2;
    }

    memset(&kud, 0, sizeof(kud));
    kud.sin_family = AF_INET;
    kud.sin_addr.s_addr = *((in_addr_t*)hent->h_addr_list[0]);
    kud.sin_port = htons(broadcast_port);

    printf("Gateway address is %s:%u\n", inet_ntoa(kud.sin_addr), broadcast_port);

    res = bind(opts->tcp_sock, (const struct sockaddr*) & kud, sizeof(kud));
    if (res < 0) {
        perror("bind()");
        return -2;
    }

    res = listen(opts->tcp_sock, 10); //connect(opts->tcp_sock, (struct sockaddr *)&kud, sizeof(kud));
    if (res < 0) {
        perror("listen()");
        return -3;
    }

    return 0;
}


static void*
agps_thread_main(void* arg)
{
    printf("AGPS thread start\n");
    nwy_loc_location_type_t loc_info;

    for (;;) {
        int result = nwy_loc_get_curr_location(&loc_info, 10);
        if (0 != result) {
            if (-703 == result) {
                printf("nwy_loc_get_curr_location() timeout\n");
            }
            else {
                printf("-----\nnwy_loc_get_curr_location() failed, returned value was %d\n", result);
                printf("Trying to recover\n-----\n");
            }
        }
    }
}


static void
broadcast(options_t* opts)
{
    int len, res;
    char buf[512];
    time_t tm = time(NULL);
    char strtime[64];

    strftime(strtime, sizeof(strtime), "%d-%m-%Y %H:%M:%S", localtime(&tm));

    //printf("\0x1B[33mBroadcasting announce\0x1B[37m\n");
    printf("%s Broadcasting announce\n", strtime);

    len = snprintf(buf, sizeof(buf),
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<info>\n"
        "<URL>%s:%d</URL>\n"
        "<UUID>%s</UUID>\n"
        "<TTL>%d</TTL>\n"
        "</info>\r\n", GATEWAY_ADDRESS, 19001, opts->uuid, opts->broadcast_period * 2
    );

    res = sendto(opts->udp_broadcast, buf, len, 0, (const struct sockaddr*) & opts->baddr, sizeof(opts->baddr));
    if (res < 0) {
        perror("sendto()");
    }
}


static void
receive_command(options_t* opts, fd_set* fds)
{
    char buf[8192];
    int res;
    struct sockaddr_in sin;
    socklen_t sinsize = sizeof(sin);

#if 0
    if (FD_ISSET(opts->tcp_sock, fds)) {
        res = recv(opts->tcp_sock, buf, sizeof(buf), 0);
        if (res < 0)
            perror("recv()");
        else if (0 == res) {
            printf("Other side closed connection\n");
        }
        printf("Received %d bytes\n", res);
    }
#endif

    if (FD_ISSET(opts->udp_broadcast, fds)) {
        sin.sin_family = AF_INET;
        sin.sin_addr.s_addr = opts->baddr.sin_addr.s_addr;
        sin.sin_port = htons(19001);
        res = recvfrom(opts->udp_broadcast, buf, sizeof(buf), 0, (struct sockaddr*) & sin, &sinsize);
        if (res < 0)
            perror("recvfrom()");

        printf("Datagram received (%d bytes) from %s\n", res, inet_ntoa(sin.sin_addr));
    }

    if (res >= 0) {
        buf[res] = 0;
        printf("%s\n", buf);
    }
}


char*
get_file_contents(const char* fn)
{
    FILE* fp = fopen(fn, "rb");
    char* data;
    long flen;

    if (NULL == fp) {
        printf("%s: ", fn);
        perror("fopen()");
        return strdup("");
    }

    fseek(fp, 0, SEEK_END);
    flen = ftell(fp);

    printf("Reading %s (%ld bytes)\n", fn, flen);

    data = (char*)malloc(flen + 3);
    if (NULL == data) {
        data = strdup("");
        perror("malloc()");
    }
    else {
        fseek(fp, 0, SEEK_SET);
        fread(data, sizeof(char), flen, fp);

        data[flen++] = '\r';
        data[flen++] = '\n';
        data[flen++] = '\0';
    }

    fclose(fp);
    return data;
}


enum {
    XML_INFO = 0,
    XML_SMS,
    XML_ROUTER,
    XML_QUERY_STATE_VARIABLE,
    //
    XML_MAX
};


char* xmls[XML_MAX];


void
read_xmls()
{
    xmls[XML_INFO] = get_file_contents("/data/xml/info.xml");
    xmls[XML_SMS] = get_file_contents("/data/xml/sms.xml");
    xmls[XML_ROUTER] = get_file_contents("/data/xml/routing.xml");
    xmls[XML_QUERY_STATE_VARIABLE] = get_file_contents("/data/xml/queryvariable.xml");
}


static void
queryStateVariable(options_t* opts)
{
    char value[256];

    sprintf(value, "%s", "Пока нет");
    _sendbuflen = sprintf(_sendbuf, xmls[XML_QUERY_STATE_VARIABLE], opts->uuid, opts->xml_variable, value, opts->xml_variable);
    _sendbuf[_sendbuflen] = 0;
    printf(_sendbuf);
}


static void
process_get(options_t* opts)
{
    char* p = strchr(opts->r_uuid, ':');

    if (NULL == p) {
        switch (opts->elem) {
        case XML_BODY:
            if (XML_CMD_QUERY_STATE_VARIABLE == opts->xml_cmd) {
                queryStateVariable(opts);
            }
            break;
        default:
            _sendbuflen = sprintf(_sendbuf, xmls[XML_INFO], opts->uuid);
            break;
        }

    }
    else {
        ++p;
        if (0 == strcasecmp(p, "SMSG")) {
            _sendbuflen = sprintf(_sendbuf, xmls[XML_SMS], opts->r_uuid);
        }
        else if (0 == strcasecmp(p, "ROUTING")) {
            _sendbuflen = sprintf(_sendbuf, xmls[XML_ROUTER], opts->r_uuid);
        }
    }
}


static void
action_send_sms(options_t* opts)
{
    int webs;
    struct hostent* hent;
    struct sockaddr_in addr;
    struct in_addr* localhost;
    char req[1024];
    int reqlen;

    reqlen = snprintf(req, sizeof(req),
        "POST / HTTP/1.1\r\n"
        "Connection: close\r\n"
        "Content-type: application/x-www-form-urlencoded\r\n"
        "\r\n"
        "SMS_send\n"
        "%s %s\r\n", opts->phone_number, opts->sms_text
    );

    hent = gethostbyname("localhost");
    if (NULL == hent) {
        herror("gethostbyname()");
        return;
    }

    localhost = (struct in_addr*)hent->h_addr_list[0];

    printf("IP=%s\n", inet_ntoa(*localhost));

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = localhost->s_addr;
    addr.sin_port = htons(80);

    webs = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (webs < 0) {
        perror("socket()");
        return;
    }

    if (connect(webs, (struct sockaddr*) & addr, sizeof(addr)) < 0) {
        perror("connect()");
        return;
    }

    if (send(webs, req, reqlen, 0) < 0) {
        perror("send()");
    }

    int len = recv(webs, req, sizeof(req), 0);
    if (len < 0) {
        perror("recv()");
    }
    else {
        req[len] = 0;
        printf("RECV: %s\n", req);
    }

    shutdown(webs, SHUT_RDWR);
    close(webs);
}


static void XMLCALL
process_character_data(void* userdata, const XML_Char* text, int len)
{
    options_t* opts = (options_t*)userdata;
    char temp[1024];

    switch (opts->elem) {
    case XML_UUID:
        strncpy(opts->r_uuid, text, len);
        opts->r_uuid[len] = 0;
        printf("UUID = %s\n", opts->r_uuid);
        break;
    case XML_NAME:
        strncpy(temp, text, len);
        temp[len] = 0;
        printf("Action name = %s\n", temp);
        memset(opts->phone_number, 0, sizeof(opts->phone_number));
        memset(opts->sms_text, 0, sizeof(opts->sms_text));
        opts->xml_action = action_send_sms;
        break;
    case XML_VALUE:
        strncpy(temp, text, len);
        temp[len] = 0;
        printf("SET %s = %s\n", opts->xml_variable, temp);
        if (0 == strcasecmp("telno", opts->xml_variable)) {
            strcpy(opts->phone_number, temp);
            printf("telno %s\n", opts->phone_number);
        }
        else if (0 == strcasecmp("text", opts->xml_variable)) {
            strcpy(opts->sms_text, temp);
            printf("text %s\n", opts->sms_text);
        }
        else if (0 == strcasecmp("pin", opts->xml_variable) && 4 == len) {
            opts->pin[0] = text[0];
            opts->pin[1] = text[1];
            opts->pin[2] = text[2];
            opts->pin[3] = text[3];
        }
        break;
    }
}


static void XMLCALL
process_element_start(void* userdata, const XML_Char* name, const XML_Char** attrs)
{
    options_t* opts = (options_t*)userdata;
    int i;

    if (0 == opts->level) {
        opts->xml_cmd = XML_CMD_NONE;
    }

    if (1 == opts->level) {
        if (0 == strcasecmp(name, "queryStateVariable")) {
            opts->xml_cmd = XML_CMD_QUERY_STATE_VARIABLE;
            printf("Mode = %d\n", opts->xml_cmd);
        }
        else if (0 == strcasecmp(name, "action")) {
            opts->xml_cmd = XML_CMD_ACTION;
            printf("ACTION\n");
        }
    }

    if (2 == opts->level) {
        switch (opts->xml_cmd) {
        case XML_CMD_QUERY_STATE_VARIABLE:
            memset(opts->xml_variable, 0, sizeof(opts->xml_variable));
            strcpy(opts->xml_variable, name);
            break;

        case XML_CMD_ACTION:
            if (0 == strcasecmp(name, "name")) {
                opts->elem = XML_NAME;
                //opts->xml_cmd = XML_CMD_ACTION_NAME;
            }
            else if (0 == strcasecmp(name, "argumentList")) {
                opts->xml_cmd = XML_CMD_ACTION_ARGS;
                printf("Expecting action argument list\n");
            }
            break;
        }
    }

    if (3 == opts->level) {
        if (XML_CMD_ACTION_ARGS == opts->xml_cmd) {
            memset(opts->xml_variable, 0, sizeof(opts->xml_variable));
            strcpy(opts->xml_variable, name);
            printf("Arg: %s\n", name);
        }
    }


    if (4 == opts->level) {
        if (XML_CMD_ACTION_ARGS == opts->xml_cmd && 0 == strcasecmp("value", name)) {
            opts->elem = XML_VALUE;
        }
        else {
            opts->elem = XML_NONE;
        }
    }

    printf("%d %s\n", opts->level, name);

    if (0 == strcasecmp(name, "get")) {
        printf("XML_GET\n");
        opts->elem = XML_GET;
    }
    else if (0 == strcasecmp(name, "UUID")) {
        opts->elem = XML_UUID;
        printf("XML_UUID\n");
    }
    else if (0 == strcasecmp(name, "body")) {
        opts->elem = XML_BODY;
        printf("XML_BODY\n");
        if (NULL != attrs) {
            for (i = 0; NULL != attrs[i]; i += 2) {
                if (0 == strcasecmp("urn", attrs[i])) {
                    memset(opts->r_uuid, 0, sizeof(opts->r_uuid));
                    strcpy(opts->r_uuid, attrs[i + 1]);
                    printf("R_UUID = %s\n", attrs[i + 1]);
                }
            }
        }
    }

    ++opts->level;
}


static void XMLCALL
process_element_end(void* userdata, const XML_Char* name)
{
    options_t* opts = (options_t*)userdata;

    (void)name;

    if (--opts->level == 0) {
        process_get(opts);
    }
}


static void*
network_thread_main(void* arg)
{
    options_t* opts = (options_t*)arg;
    fd_set rfds, wfds;
    struct timeval tm;
    int res, maxsock;
    time_t last_ts, now;
    int i, sock;
    char buf[8192];
    XML_Parser parser;

    parser = XML_ParserCreate(NULL);

    printf("Network thread start\n");

    read_xmls();

    last_ts = time(NULL);
    for (;;) {
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);

        FD_SET(opts->udp_broadcast, &wfds);
        FD_SET(opts->tcp_sock, &rfds);

        maxsock = MAX(opts->udp_broadcast, opts->tcp_sock);

        tm.tv_sec = opts->broadcast_period;
        tm.tv_usec = 0;

        res = select(maxsock + 1, &rfds, &wfds, NULL, &tm);
        now = time(NULL);
        if (res < 0) {
            if (EINTR == errno)
                continue;
        }
        else if (res == 0) { // timeout reached
            broadcast(opts);
            last_ts = now;
        }
        else {
            if (last_ts + opts->broadcast_period < now) { // timeout reached
                broadcast(opts);
                last_ts = now;
            }

            if (FD_ISSET(opts->tcp_sock, &rfds)) {
                struct sockaddr_in in_addr;
                socklen_t len = sizeof(in_addr);

                sock = accept(opts->tcp_sock, (struct sockaddr*) & in_addr, &len);
                printf("Accepted connection from %s:%u\n", inet_ntoa(in_addr.sin_addr), ntohs(in_addr.sin_port));

                res = recv(sock, buf, sizeof(buf), 0);
                if (res > 0) {
                    memset(_sendbuf, 0, sizeof(_sendbuf));
                    _sendbuflen = 0;

                    //printf("%s", buf);
                    opts->level = 0;
                    opts->xml_action = NULL;

                    XML_SetUserData(parser, opts);
                    XML_SetStartElementHandler(parser, process_element_start);
                    XML_SetEndElementHandler(parser, process_element_end);
                    XML_SetCharacterDataHandler(parser, process_character_data);
                    XML_Parse(parser, buf, res, TRUE);

                    process_get(opts);

                    if (NULL != opts->xml_action) {
                        opts->xml_action(opts);
                    }

                    //printf("%s", _sendbuf);
                    if (0 != _sendbuflen) {
                        res = send(sock, _sendbuf, _sendbuflen, 0);
                    }

                    XML_ParserReset(parser, NULL);
                }

                shutdown(sock, SHUT_RDWR);
                close(sock);
            }
#if 0
            if (FD_ISSET(opts->udp_broadcast, &rfds)) {
                receive_command(opts, &rfds);
            }
#endif
        }
    }

    return NULL;
}


static int
parse_command_line(options_t* opts, int argc, char* argv[])
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
    char* end;

    while ((opt = getopt_long(argc, argv, _shortopts, _options, &idx)) >= 0) {
        if (idx < 0) {
            for (idx = 0; '\0' != _shortopts_chars[idx]; ++idx)
                if (_shortopts_chars[idx] == opt)
                    break;

            if (idx < 0)
                return -1;
        }

        switch (idx) {
        case 0: // nodaemon
            opts->go_daemon = TRUE;
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
            if ('\0' == *end) {
                opts->uart_baud_rate = atoi(optarg);
            }
            else {
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
app_init(options_t* opts)
{
    int res;
    if (0 != network_init(opts)) {
        printf("Network init failed\n");
        exit(EXIT_FAILURE);
    }

    opts->uart_fd = uart_init(opts->uart_tty, opts->uart_baud_rate);
    if (-1 == opts->uart_fd) {
        printf("UART init failed\n");
        exit(EXIT_FAILURE);
    }

    if (opts->gprs_enabled) {
        /* Get SIM status */
        printf("Querying SIM (SLOT 1) status\n");
        opts->sim_status = nwy_sim_get_card_status(NWY_SIM_ID_SLOT_1);
        if (opts->sim_status < 0) {
            printf("Error (%d) querying SIM (SLOT 1) status. GPRS service disabled.\n", opts->sim_status);
            opts->sim_status = NWY_SIM_NOT_INSERTED;
            opts->gprs_enabled = FALSE;
        }
        else {
            switch (opts->sim_status) {
            case NWY_SIM_NOT_INSERTED:
                printf("SIM card not present (SLOT 1). Disabling SMS and GPRS services.\n");
                opts->gprs_enabled = FALSE;
                break;

            case NWY_SIM_PIN_REQ:
                res = nwy_sim_verify_pin(NWY_SIM_ID_SLOT_1, opts->pin);
                if (res < 0) {
                    printf("SIM PIN verify failed. Error code is %d\n", res);
                    opts->gprs_enabled = FALSE;
                };
                break;
            }
        }
    }

    opts->modem_fd = modem_init(opts->modem_tty, opts->uart_baud_rate);
    if (-1 == opts->modem_fd) {
        printf("MODEM init failed\n");
        exit(EXIT_FAILURE);
    }

#if 0
    if (opts->gps_enabled) {
        if (0 != agps_init()) {
            printf("AGPS init failed\n");
            //exit(EXIT_FAILURE);
        }
    }
#endif

    nwy_gpio_set_dir(NWY_GPIO_78, NWY_OUTPUT);
    nwy_gpio_set_val(NWY_GPIO_78, NWY_HIGH);

    return 0;
}


// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------

/*---------------------------Macro Definition---------------------------*/
#define PORT 80               //Требует Rут права

#undef LOG_FORMAT
#define LOG_FORMAT       LOG_SIMPLE
#undef LOG_MSG_TAG
#define LOG_MSG_TAG      "AT"
#undef LOG_MSG_LEVEL
#define LOG_MSG_LEVEL    LOG_DEBUG
#undef LOG_MSG_MASK
#define LOG_MSG_MASK     LOG_MASK_STD

#define NWY_AT_PORT      "/dev/smd9"
#define BUFLEN 512             //Max length of buffer
#define SERVER "10.7.254.6"    //Адресс локальной машины пк связанной с железкой сейчас
#define MAX_SMS_IN_PRIORITY 20 //Предельное количество смс в каждом приоритете. Приоритеты 1-7
#define MAX_SMS_LENGTH 160     //Предельное количество символов в смс, годится только для отправки латиницы


char buffer[65000] = { 0 };    //Буфер для вычитывания файлов и принятия внешних запросов
//Переменные, для вычитывания данных из внешнего запроса
char method[10] = { 0 },                //Метод Post, get или что то еще
fileadrr[100] = { 0 },                //Адрес файла к которому обращение
filetype[10] = { 0 },                //Расширение файла
postcomand[20] = { 0 },                //Команда для сервера через POST
postbody[1000] = { 0 };                //Тело Post запроса
//Переменные, для хранения основных параметров монитора, остальные ищи в options_t в utils.h
int num_sput_val = 0,
carrige_mileage_val = 0,
last_mileage_val = 0,
power_type_val = 0;

// char country_cod[10], operator_cod[10];
int hh, mm, ss, ms;                 //Переменные времени от GPS идут в системное время
char fix_state;                     //Состояние фиксации спутниками нет/плоскость/3-х мерное пространство
int lat_D, lon_D;
float lat_M = 0, lon_M = 0, lat = 0, lon = 0;
char lat_sign = '\0', lon_sign = '\0';

char phone_num[13], sms_text[500] = { '\0' }; //Переменные для работы с смс
char recv_phone[13], recv_text[500] = { '\0' };
bool recv_flag = false;
bool queue_flag = false;

char web_log[64000] = { 0 };
char inet_web[16] = { 0 };          //ifconfig bridge0 inet 192.168.0.0 netmask 255.255.255.0
char netmask_web[16] = { 0 };
char gps_time_web[5] = { 0 };

int opt = 1, rc;
int user_id = 0;                   //2 - Admin, 1 - Viewer, 0 - Her kakoi-to
int ret;
time_t start, end;
int server_fd, new_tcp_socket, valread;  //

struct sockaddr_in address;
int addrlen = sizeof(address);

void Header_Parse();                                //Функция которая разбирает ключевые заголовки из buffer и складывает в предназначенные для этого массивы
void Start_Socket();                                //Socket+bind+listen
void At_init(void* web_opts);                       //Инициализация для работы с AT
void send_at_cmd(char* at_com, void* web_opts);     //Отправка АТ команды
static void at_free(char** p);
//static void* TCP_server (void *arg);
//static void* UDP_server (void *arg);
static void del_sms_num(char* but_type, int num, void* web_opts);
static void* HeartBit(void* arg);
static int do_send_sms(char* number, int encoding, int length, char* context, int async, void* opts_sms, int sms_priority);
static void test_sms_evt_handler(nwy_mt_sms_event_t ind_type, void* ind_struct);
int Read_smsFrom_txt(const char* file, void* web_opts);
int Write_smsTo_txt(const char* file, void* web_opts);

// int main(int argc, char const *argv[])
// {
//         sprintf(web_log, "%s%s", web_log, "Start systems...\\n");
//         system("ifconfig bridge0 inet 10.7.254.10 netmask 255.255.255.0\n");
//         system("ethtool -s eth0 speed 100 duplex full autoneg on\n");
//
//         pthread_t thread, thread2, thread3;
//         pthread_create(&thread, NULL, TCP_server, "1");
//         pthread_create(&thread2, NULL, UDP_server, "2");
//         pthread_create(&thread3, NULL, HeartBit, "3");
//         while(1)
//         {
//                 printf("\033[92mNORMAL WORK MAIN\033[0m\n");
//                 sleep(5);
//         }
//         return 0;
// }

void Start_Socket()
{
    //Создание дескриптера сокета
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    start = time(NULL);
    printf("SOCKET: OK\n");
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;                             //Adress Family - Семейство адрессов для интернет домена либой IP сети
    address.sin_addr.s_addr = INADDR_ANY;                     //Если комп имеет несколько сетевых адрессов, и готовы соединяться с клиентом по любому из них
    address.sin_port = htons(PORT);                         //При указании IP-адреса и номера порта необходимо преобразовать число из порядка хоста в сетевой
    //htons (Host TO Network Short) и htonl (Host TO Network Long). Обратное преобразование выполняют функции ntohs и ntohl

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*) & address, sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    printf("BIND: OK\n");
    if (listen(server_fd, 10) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    printf("LISTEN: OK\n");
}

void Header_Parse()
{
    int i = 0, j = 0;
    fileadrr[0] = '\0';                                  //Костыль для корректной работе при перезагрузке страницы
    filetype[0] = '\0';
    postcomand[0] = '\0';
    postbody[0] = '\0';
    while (buffer[i] != ' ')                             //Обработка параметров первой строки
    {                                                    //Вида "GET / HTTP1.1"
        if (buffer[0] == '\0') break;
        method[j] = buffer[i];                       //POST or GET
        i++;
        j++;
    }
    method[j] = '\0';
    i++; j = 0; i++;
    while (buffer[i] != ' ')
    {
        if (buffer[i] == '\0') break;
        fileadrr[j] = buffer[i];                      //"index.sFile"?
        i++;
        j++;
        if (buffer[i] == '.')                         //Если в запрсе точка
        {
            fileadrr[j] = '\0';                   //закончили читать имя файла
            i++; j = 0;
            while (buffer[i] != ' ')              //читаем расширение файла
            {
                filetype[j] = buffer[i];      //"index.sFile"?
                i++;
                j++;
            }
            filetype[j] = '\0';
            break;
        }
        else if (buffer[i] == ' ')
        {
            fileadrr[j] = '\0';
            break;
        }
    }
    if (method[0] == 'P')
    {
        while ((buffer[i] != '\n') || (buffer[i - 2] != '\n'))
        {
            i++;
        }
        i++;
        j = 0;
        while (buffer[i] != '\n')
        {
            postcomand[j] = buffer[i];
            i++;
            j++;
        }
        postcomand[j] = '\0';
        i++; j = 0;
        while (buffer[i] != '\0')
        {
            postbody[j] = buffer[i];
            i++;
            j++;
        }
        postbody[j] = '\0';
    }
    if (fileadrr[0] == '\0') { fileadrr[0] = 'i'; fileadrr[1] = 'n'; fileadrr[2] = 'd'; fileadrr[3] = 'e'; fileadrr[4] = 'x'; fileadrr[5] = '\0'; }
    if (filetype[0] == '\0') { filetype[0] = 'h'; filetype[1] = 't'; filetype[2] = 'm'; filetype[3] = 'l'; filetype[4] = '\0'; }
}

void At_init(void* web_opts)
{
    options_t* opts = (options_t*)web_opts;
    ret = nwy_at_port_init(NWY_AT_PORT);
    if (ret != 0)
    {
        LOGI("Init port error \n", NWY_AT_PORT);
    }
    else
    {
        send_at_cmd("AT$MYGPSPWR=1\0", opts);
        printf("AT: OK\n");
    }
}

void send_at_cmd(char* at_com, void* web_opts)
{
    options_t* opts = (options_t*)web_opts;
    char* p_resp = NULL;
    char* p_result = NULL;
    int ret;
    int i = 0, j = 0;
    ret = nwy_at_send_cmd(at_com, &p_resp, &p_result);
    if (ret != 0)
    {
        printf("Send at cmd %s failed\n", at_com);
        return;
    }
    if (p_resp != NULL)
    {
        #ifdef WEBPOSTGETINCONSOLE
        printf("Recv at response:\n%s\n%s\n", p_resp, p_result);
        #endif

        if (!strcmp(at_com, "AT+CSQ\0"))
        {
            while (p_resp[i] != ' ')
            {
                i++;
            }
            i++;
            while (p_resp[i] != ',')
            {
                opts->rssi[j] = p_resp[i];
                i++; j++;
            }
            opts->rssi[j] = '\0';
            int rssi_val = -113 + atoi(opts->rssi) * 2;
            sprintf(opts->rssi, "%d", rssi_val);
        }
        else if (!strcmp(at_com, "AT+CIMI\0"))
        {
            while (p_resp[i] != ' ')
            {
                i++;
            }
            i++;
            while (p_resp[i] != '\0')
            {
                opts->imsi[j] = p_resp[i];
                if (j <= 2)
                {
                    opts->country_cod[j] = p_resp[i];
                    opts->country_cod[j + 1] = '\0';
                }
                if (j == 3 || j == 4)
                {
                    opts->operator_cod[j - 3] = p_resp[i];
                    opts->operator_cod[j - 2] = '\0';
                }
                i++; j++;
            }
            switch (atoi(opts->country_cod))
            {
            case 250:
                sprintf(opts->country_cod, "RUS\0");
                break;
            default:
                sprintf(opts->country_cod, "\0");
                break;
            }
            switch (atoi(opts->operator_cod))
            {
            case 1:
                sprintf(opts->operator_cod, "MTS\0");
                break;
            case 2:
                sprintf(opts->operator_cod, "Megafon\0");
                break;
            case 11:
                sprintf(opts->operator_cod, "Yota\0");
                break;
            case 14:
                sprintf(opts->operator_cod, "Megafon\0");
                break;
            case 20:
                sprintf(opts->operator_cod, "Tele2\0");
                break;
            case 28:
                sprintf(opts->operator_cod, "Beeline\0");
                break;
            case 62:
                sprintf(opts->operator_cod, "Tinkoff\0");
                break;
            case 99:
                sprintf(opts->operator_cod, "Beeline\0");
                break;
            default:
                sprintf(opts->operator_cod, "\0");
                break;
            }
            opts->imsi[j] = '\0';
        }
        else if (!strcmp(at_com, "AT+CGSN\0"))
        {
            while (p_resp[i] != ' ')
            {
                i++;
            }
            i++;
            while (p_resp[i] != '\0')
            {
                opts->imei[j] = p_resp[i];
                i++; j++;
            }
            opts->imei[j] = '\0';
        }
        else if (!strcmp(at_com, "AT$MYGPSPOS=3\0"))
        {
            sscanf(p_resp, "$MYGPSPOS: $GPRMC,%2d%2d%2d.%2d,%c,%2d%f,%c,%3d%f,%c", &hh, &mm, &ss, &ms, &fix_state, &lat_D, &lat_M, &lat_sign, &lon_D, &lon_M, &lon_sign);
            if (lat_sign == '\0' || lon_sign == '\0')
            {
                lat = 0;
                lon = 0;
                lat_sign = '-';
                lon_sign = '-';
            }
            else
            {
                lat = lat_D + lat_M / 60;
                lon = lon_D + lon_M / 60;
            }
        }
        else if (!strcmp(at_com, "AT$MYGPSPOS=1\0"))
        {
            while (p_resp[i] != ',')
            {
                i++;
            }
            i++;
            while (p_resp[i] != ',')
            {
                i++;
            }
            i++;
            while (p_resp[i] != ',')
            {
                if (p_resp[i] == '1')
                {
                    sprintf(opts->threed_fix, "invalid\0");
                }
                if (p_resp[i] == '2')
                {
                    sprintf(opts->threed_fix, "2D FIX\0");
                }
                if (p_resp[i] == '3')
                {
                    sprintf(opts->threed_fix, "3D FIX\0");
                }
                i++;
            }
            i++;
            num_sput_val = 0;
            while (num_sput_val < 12)
            {
                if (p_resp[i] == ',')
                {
                    num_sput_val++;
                    if (p_resp[i + 1] == ',')
                    {
                        break;
                    }
                }
                i++;
            }
            opts->gps_cords[j] = ' ';
            j++;
            while (p_resp[i] != ',')
            {
                i++;
            }
            i++;
            while (p_resp[i] != ',')
            {
                opts->gps_cords[j] = p_resp[i];
                i++; j++;
                if (j == 15)
                {
                    j++;
                    j++;
                }
                if (p_resp[i] == '.')
                {
                    i++;
                }
            }
            opts->gps_cords[j] = '\0';
        }
        else if (!strcmp(at_com, "AT$MYSYSINFO\0"))
        {
            while (p_resp[i] != ' ')
            {
                i++;
            }
            i++;
            switch (atoi(&p_resp[i]))
            {
            case 0:
                sprintf(opts->mobile_data, "No Signal\0");
                break;
            case 2:
                sprintf(opts->mobile_data, "2G\0");
                break;
            case 3:
                sprintf(opts->mobile_data, "3G\0");
                break;
            case 4:
                sprintf(opts->mobile_data, "LTE\0");
                break;
            }
        }
    }
    else{
    #ifdef WEBPOSTGETINCONSOLE
        printf("Recv at response: %s\n", p_result);
    #endif
    }
    #ifdef WEBPOSTGETINCONSOLE
    printf("Send at cmd %s: OK\n", "at_com");
    #endif
    at_free(&p_resp);
    at_free(&p_result);
    return;
}

static void at_free(char** p)
{
    if ((*p) != NULL)
    {
        free(*p);
        (*p) = NULL;
    }
}


static void* tcp_web_thread_main(void* arg)
{
    options_t* opts = (options_t*)arg;
    char* at_com = malloc(sizeof(char) * 100);
    int i = 0;
    int j = 0;
    FILE* sFile;                            //Открытие файлов
    long int nFileLen;                      //Сюда пишем позицию
    signal(SIGPIPE, SIG_IGN);                //Игнорим ситуацию если пакет послан, но не был принят
    Start_Socket();                         //Запуск Socket+bind+listen
    At_init(opts);                          //Инициализация для работы с AT

    end = time(NULL);

    char sss[2], mmm[2], hhh[2];            //складываем в строки секунды часы и минуты
    (((int)difftime(end, start) % 60 < 10) ? sprintf(sss, "0%d", (int)difftime(end, start) % 60) : sprintf(sss, "%d", (int)difftime(end, start) % 60));
    (((int)(difftime(end, start) / 60) % 60 < 10) ? sprintf(mmm, "0%d", (int)(difftime(end, start) / 60) % 60) : sprintf(mmm, "%d", (int)(difftime(end, start) / 60) % 60));
    (((int)(difftime(end, start) / 3600) % 60 < 10) ? sprintf(hhh, "0%d", (int)(difftime(end, start) / 3600) % 60) : sprintf(hhh, "%d", (int)(difftime(end, start) / 3600) % 60));
    sprintf(opts->up_time_string, "%s : %s : %s\0", hhh, mmm, sss);

    int s_p;                           //выставляем нулевое количкстов всех сообщений, пополнится само при чтении файла sms_memory.txt
    for (s_p = 7; s_p >= 0; s_p--)
    {
        opts->sended[s_p].j = 0;
        opts->deleted[s_p].j = 0;
        opts->queue[s_p].j = 0;
        opts->recved[s_p].j = 0;
    }
    /*opts->sended[3].j = 3;
    //Тестовое заполнение массива сообщений
    for (i = opts->sended[3].j-1; i >= 0; i--)
    {       printf ("i = %d\n", i);
            //int j = opts->sended[s_p].j;
            sprintf(opts->sended[3].phone[i], "%s\0", "+7910123456"); printf("%s\n", opts->sended[3].phone[i]); sprintf(opts->sended[3].text[i], "%s%d\0", "Text", i); printf("%s\n", opts->sended[3].text[i]);
    }
    opts->sended[7].j = 4;
    //Тестовое заполнение массива сообщений
    for (i = opts->sended[7].j-1; i >= 0; i--)
    {       printf ("i = %d\n", i);
            //int j = opts->sended[s_p].j;
            sprintf(opts->sended[7].phone[i], "%s\0", "+7910123456"); printf("%s\n", opts->sended[7].phone[i]); sprintf(opts->sended[7].text[i], "%s%d\0", "Text", i); printf("%s\n", opts->sended[7].text[i]);
    }
    opts->deleted[5].j = 5;
    for (i = opts->deleted[5].j-1; i >= 0; i--)
    {       printf ("i = %d\n", i);
            //int j = opts->sended[s_p].j;
            sprintf(opts->deleted[5].phone[i], "%s\0", "+7910123456"); printf("%s\n", opts->deleted[5].phone[i]); sprintf(opts->deleted[5].text[i], "%s%d\0", "Text", i); printf("%s\n", opts->deleted[5].text[i]);
    }*/
    nwy_sms_add_mtmessage_handler(test_sms_evt_handler, NULL);    //Добавить обработчик события получения смс
    Read_smsFrom_txt("sms_memory.txt\0", opts);
    int count_to_save_sms = 0;
    //Работа с клиентом
    while (1)
    {
        new_tcp_socket = accept(server_fd, NULL, NULL);                                          //(struct sockaddr *)&address, (socklen_t*)&addrlen);
        if (new_tcp_socket == -1)
        {
            perror("accept");
#ifdef WEBPOSTGETINCONSOLE
            printf("\033[91mACCEPT:\033[0m FAIL, %f seconds from start\n", difftime(end, start));
#endif
            continue;                                                                        //exit(EXIT_FAILURE);
        }
        end = time(NULL);
#ifdef WEBPOSTGETINCONSOLE
        printf("\033[91mACCEPT:\033[0m OK, %f seconds from start\n", difftime(end, start));
#endif
        valread = recv(new_tcp_socket, buffer, sizeof(buffer) - 1, 0);                              //Разбираем запрос от браузера
        end = time(NULL);
#ifdef WEBPOSTGETINCONSOLE
        printf("\033[91mRecv:\033[0m End, %f seconds from start\n", difftime(end, start));
#endif
        if (valread <= 0)
        {
            send(new_tcp_socket, "HTTP/1.1 400\r\n", strlen("HTTP/1.1 400\r\n"), 0);
            //printf("Fatalerror\n");
            close(new_tcp_socket);
            continue;
        }
        buffer[valread] = '\0';                                                 //Добавляем конец строки на позиции конца строки
        #ifdef WEBPOSTGETINCONSOLE
        printf("\033[91mREAD:\033[0m OK, %d symbols\n", valread);
        #endif
        Header_Parse();
        //Вывод на экран всего что там напарсилось
        #ifdef WEBPOSTGETINCONSOLE
        printf("%s\n", buffer);
        printf("\033[92mMethod:\033[0m %s\n", method);
        printf("\033[92mAddr:\033[0m %s \n", fileadrr);
        printf("\033[92mType:\033[0m %s\n", filetype);
        printf("\033[92mComand:\033[0m %s\n", postcomand);
        printf("\033[92mBody:\033[0m %s\n", postbody);
        #endif
        sprintf(fileadrr, "%s.%s", fileadrr, filetype);                                                 //Собираем целый адресс из имени и расширения
        #ifdef WEBPOSTGETINCONSOLE
        printf("\033[92mAddr:\033[0m %s \n", fileadrr);
        #endif

        if (method[0] == 'P' && method[1] == 'O' && method[2] == 'S' && method[3] == 'T')
        {
            //Авторизация под пользователем
            if (!strcmp(postcomand, "validate\0") && (!strcmp(postbody, "Admin 12345\0")))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                user_id = 2;
            }
            else if (!strcmp(postcomand, "validate\0") && (!strcmp(postbody, "User 123\0")))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                user_id = 1;
            }
            else if (!strcmp(postcomand, "logout\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                if (user_id == 2) {
                    sprintf(web_log, "%s[%s] %s", web_log, opts->up_time_string, "Admin is unlogged\\n");
                }
                else if (user_id == 1) {
                    sprintf(web_log, "%s[%s] %s", web_log, opts->up_time_string, "User is unlogged\\n");
                }
                user_id = 0;
            }
            else if (!strcmp(postcomand, "SMS_send\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                int i = 0, j = 0;
                while (postbody[i] != ' ')
                {
                    phone_num[j] = postbody[i];
                    i++;
                    j++;
                }
                phone_num[j] = '\0';
                j = 0; i++;
                while (postbody[i] != '\0')
                {
                    sms_text[j] = postbody[i];
                    i++;
                    j++;
                }
                sms_text[j] = '\0';
                //squeeze (sms_text, '\n'); //удалим запрещенный \n
                printf("Num: %s\n SMS: %s\n Len: %d\n", phone_num, sms_text, strlen(sms_text));
                do_send_sms(phone_num, 0, strlen(sms_text), sms_text, 0, opts, 3);
            }
            else if (!strcmp(postcomand, "SMS_save\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                Write_smsTo_txt("sms_memory.txt\0", opts);
                #ifdef WEBPOSTGETINCONSOLE
                printf("SMS_saving: OK\n");
                #endif
            }
            else if (!strcmp(postcomand, "save_netstat\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                int i = 0, j = 0;
                while (postbody[i] != ' ')
                {
                    inet_web[j] = postbody[i];
                    i++;
                    j++;
                }
                inet_web[j] = '\0';
                j = 0; i++;
                while (postbody[i] != ' ')
                {
                    netmask_web[j] = postbody[i];
                    i++;
                    j++;
                }
                netmask_web[j] = '\0';
                j = 0; i++;
                while (postbody[i] != '\0')
                {
                    gps_time_web[j] = postbody[i];
                    i++;
                    j++;
                }
                gps_time_web[j] = '\0';
                printf("inet: %s\nnetmask: %s\ngps_time: %s\n", inet_web, netmask_web, gps_time_web);
                char sys_com[100] = { 0 };
                sprintf(sys_com, "ifconfig bridge0 inet %s netmask %s\n", inet_web, netmask_web);
                system(sys_com);
                sprintf(web_log, "%s[%s] Сетевая конфигурация обновлена!\\nip:%s\\nnetmask:%s\\n", web_log, opts->up_time_string, inet_web, netmask_web);
                //printf("%s", sys_com);
                // j = 0; i++;
            }
            else if (!strcmp(postcomand, "del_sms\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                int i = 0, j = 0;
                char but_selected[10];
                char but_num[4];
                while (postbody[i] != ' ')
                {
                    but_selected[j] = postbody[i];
                    i++;
                    j++;
                }
                but_selected[j] = '\0';
                j = 0; i++;
                while (postbody[i] != '\0')
                {
                    but_num[j] = postbody[i];
                    i++;
                    j++;
                }
                but_num[j] = '\0';
                printf("Button: %s\nBut_Num: %s\nInt_But: %d\n", but_selected, but_num, atoi(but_num));
                del_sms_num(but_selected, atoi(but_num), opts);
            }
            else
            {
                send(new_tcp_socket, "HTTP/1.1 400 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
            }
            close(new_tcp_socket);
            end = time(NULL);
            #ifdef WEBPOSTGETINCONSOLE
            printf("\033[91mCLOSE:\033[0m OK, %f seconds from start\n", difftime(end, start));
            #endif
        }



        if (method[0] == 'G' && method[1] == 'E' && method[2] == 'T')
        {
            switch (user_id)
            {
            case 0:
                if (!strcmp(fileadrr, "index.html\0") || !strcmp(filetype, "ico\0") || !strcmp(filetype, "png\0") || !strcmp(fileadrr, "blocked.html\0") || !strcmp(fileadrr, "login.js\0") || !strcmp(fileadrr, "picnic.css\0") || !strcmp(fileadrr, "sms_memory.txt"))
                {
                    printf("Guest Ok\n");
                }
                else
                {
                    sprintf(fileadrr, "404.html");
                    sprintf(filetype, "html");
                    #ifdef WEBPOSTGETINCONSOLE
                    printf("Guest sheet %s\n", fileadrr);
                    #endif
                }
                break;
            case 1:
                if (!strcmp(fileadrr, "index.html\0"))
                {
                    sprintf(fileadrr, "indexWebViewer.html");
                    sprintf(web_log, "%s[%s] %s", web_log, opts->up_time_string, "User is logged\\n");
                }
                #ifdef WEBPOSTGETINCONSOLE
                printf("Hallo User :)\n");
                #endif
                break;
            case 2:
                if (!strcmp(fileadrr, "index.html\0"))
                {
                    sprintf(fileadrr, "indexWeb.html");
                    sprintf(web_log, "%s[%s] %s", web_log, opts->up_time_string, "Admin is logged\\n");
                }
                #ifdef WEBPOSTGETINCONSOLE
                printf("Hallo Admin :)\n");
                #endif
                break;
            }


            if (send(new_tcp_socket, "HTTP/1.1 200 OK\r\n", strlen("HTTP/1.1 200 OK\r\n"), 0) == -1)
                printf("ERROR SEND\n");
            if (fileadrr[0] == '.')                                                                         //Если запрос пустой, то кидаем на стартовую страницу
            {
                send(new_tcp_socket, "Content-Type: text/html;charset=utf-8\r\n\r\n", strlen("Content-Type: text/html;charset=utf-8\r\n\r\n"), 0);
            }
            else                                                                                  //Иначе смотрим расширение запрошеного файла
            {
                if (filetype[0] == 'h' && filetype[1] == 't' && filetype[2] == 'm' && filetype[3] == 'l')
                {
                    send(new_tcp_socket, "Content-Type: text/html;charset=utf-8\r\n\r\n", strlen("Content-Type: text/html;charset=utf-8\r\n\r\n"), 0);
                }
                if (filetype[0] == 'p' && filetype[1] == 'n' && filetype[2] == 'g')
                {
                    send(new_tcp_socket, "Content-Type: image/png;charset=utf-8\r\n\r\n", strlen("Content-Type: image/png;charset=utf-8\r\n\r\n"), 0);
                }
                if (filetype[0] == 'i' && filetype[1] == 'c' && filetype[2] == 'o')
                {
                    send(new_tcp_socket, "Content-Type: image/webp;charset=utf-8\r\n\r\n", strlen("Content-Type: image/webp;charset=utf-8\r\n\r\n"), 0);
                }
                if (filetype[0] == 't' && filetype[1] == 'x' && filetype[2] == 't')
                {
                    send(new_tcp_socket, "Content-Type: text/html;charset=utf-8\r\n\r\n", strlen("Content-Type: text/html;charset=utf-8\r\n\r\n"), 0);
                }
                if (filetype[0] == 'j' && filetype[1] == 's' && filetype[2] == '\0')
                {
                    send(new_tcp_socket, "Content-Type: text/javascript;charset=utf-8\r\n\r\n", strlen("Content-Type: text/javascript;charset=utf-8\r\n\r\n"), 0);
                }
                if (filetype[0] == 'j' && filetype[1] == 's' && filetype[2] == 'o' && filetype[3] == 'n')
                {
                    send(new_tcp_socket, "Content-Type: application/json;charset=utf-8\r\n\r\n", strlen("Content-Type: application/json;charset=utf-8\r\n\r\n"), 0);
                }
                if (filetype[0] == 'c' && filetype[1] == 's' && filetype[2] == 's')
                {
                    send(new_tcp_socket, "Content-Type: text/css;charset=utf-8\r\n\r\n", strlen("Content-Type: text/css;charset=utf-8\r\n\r\n"), 0);
                }
            }
            if (!strcmp(fileadrr, "data.json\0"))
            {
                //strcpy(at_com, "AT+CSQ\0");
                send_at_cmd("AT+CSQ\0", opts);
                send_at_cmd("AT+CIMI\0", opts);
                send_at_cmd("AT+CGSN\0", opts);
                send_at_cmd("AT$MYGPSPOS=1\0", opts);
                send_at_cmd("AT$MYGPSPOS=3\0", opts);
                send_at_cmd("AT$MYSYSINFO\0", opts);
                //strcpy(at_com, "AT+CIMI\0");
                //send_at_cmd(at_com);
                end = time(NULL);
                //float time_f = difftime(end, start);
                //char sss[2], mmm[2], hhh[2];
                (((int)difftime(end, start) % 60 < 10) ? sprintf(sss, "0%d", (int)difftime(end, start) % 60) : sprintf(sss, "%d", (int)difftime(end, start) % 60));
                (((int)(difftime(end, start) / 60) % 60 < 10) ? sprintf(mmm, "0%d", (int)(difftime(end, start) / 60) % 60) : sprintf(mmm, "%d", (int)(difftime(end, start) / 60) % 60));
                (((int)(difftime(end, start) / 3600) % 60 < 10) ? sprintf(hhh, "0%d", (int)(difftime(end, start) / 3600) % 60) : sprintf(hhh, "%d", (int)(difftime(end, start) / 3600) % 60));
                sprintf(opts->up_time_string, "%s : %s : %s\0", hhh, mmm, sss);
                sprintf(buffer, "{\"rssi\" : \"%s дБм\",\n\"rsrq\" : \" - \",\n\"snr\" : \" - \",\n\"rxlen\" : \" - \"\n,\"spn\" : \" -\"\n,\"threed_fix\" : \"%s\"\n,\"gps_cords\" : \" %c %f\xC2\xB0 %c %f\xC2\xB0\"\n,\"sys_time\" : \" %d:%d:%d (GMT +3)\"\n,\"num_sput\" : \"%d\"\n,\"reg_in_mesh\" : \" %s %s\"\n,\"mobile_data\" : \"%s\"\n,\"imsi\" : \"%s\"\n,\"imei\" : \"%s\"\n,\"uptime\" : \"%s\"\n,\"carrige_mileage\" : \"%s\"\n,\"last_mileage\" : \"%s\"\n,\"power_type\" : \"%s\"\n}\0", opts->rssi, opts->threed_fix, lat_sign, lat, lon_sign, lon, hh + 3, mm, ss, num_sput_val, opts->operator_cod, opts->country_cod, opts->mobile_data, opts->imsi, opts->imei, opts->up_time_string, opts->carrige_mileage, opts->last_mileage, opts->power_type);
                send(new_tcp_socket, buffer, strlen(buffer), 0);
                at_com = NULL;
            }
            else if (!strcmp(fileadrr, "log.json\0")) {
                sprintf(buffer, "{\"log\" : \"%s\"\n}\0", web_log);
                send(new_tcp_socket, buffer, strlen(buffer), 0);
            }
            else if (!strcmp(fileadrr, "sms.json\0")) {
                /*count_to_save_sms++;
                if (count_to_save_sms > 30)
                {
                    count_to_save_sms = 0;
                    Write_smsTo_txt("sms_memory.txt\0", opts);
                }*/
                if (recv_flag == true)                             //если приходило смс, добавис его в список
                {
                    recv_flag = false;
                    //printf("FLAG IS %d\nNUM IS %s\nTEXT IS %s\n", recv_flag, recv_phone, recv_text);
                    memcpy(opts->recved[3].text[opts->recved[3].j], recv_text, 500);
                    memcpy(opts->recved[3].phone[opts->recved[3].j], recv_phone, 12);
                    if (opts->recved[3].j <= 20) {
                        opts->recved[3].j++;
                    }
                    else {
                        int z;
                        for (z = 0; z <= opts->recved[3].j - 1; z++) {
                            memcpy(opts->recved[3].phone[z], opts->recved[3].phone[z + 1], 12);
                            memcpy(opts->recved[3].text[z], opts->recved[3].text[z + 1], 500);
                        }
                    }
                    // printf("FLAG IS %d\nNUM IS %s\nTEXT IS %s\n", recv_flag, recv_phone, recv_text);
                }
                int i = 0;
                sprintf(buffer, "{\"sended\":[\0");
                for (s_p = 7; s_p >= 0; s_p--)
                {
                    for (i = opts->sended[s_p].j - 1; i >= 0; i--)
                    {
                        char one_sms[600] = { 0 };
                        //printf ("i= %d\n", i);
                        char string_whithout_quotes[500] = { 0 };
                        int ia = 0, ib = 0;
                        for (ib = 0; ib < 500; ib++)
                        {
                            if (opts->sended[s_p].text[i][ib] == '\0')
                            {
                                string_whithout_quotes[ia] = opts->sended[s_p].text[i][ib];
                                break;
                            }
                            else if (opts->sended[s_p].text[i][ib] == '\"')
                            {
                                string_whithout_quotes[ia] = '&';
                                ia++;
                                string_whithout_quotes[ia] = 'r';
                                ia++;
                                string_whithout_quotes[ia] = 'd';
                                ia++;
                                string_whithout_quotes[ia] = 'q';
                                ia++;
                                string_whithout_quotes[ia] = 'u';
                                ia++;
                                string_whithout_quotes[ia] = 'o';
                                ia++;
                                string_whithout_quotes[ia] = ';';
                                ia++;
                            }
                            else
                            {
                                string_whithout_quotes[ia] = opts->sended[s_p].text[i][ib];
                                ia++;
                            }
                        }
                        sprintf(one_sms, "\"%d %s %s\"", s_p, opts->sended[s_p].phone[i], string_whithout_quotes);
                        strcat(buffer, one_sms);
                        strcat(buffer, ", ");
                    }
                }
                if (buffer[strlen(buffer) - 2] == ',') { buffer[strlen(buffer) - 2] = '\0'; }
                strcat(buffer, "],\n\"queue\":[\0");
                for (s_p = 7; s_p >= 0; s_p--)
                {
                    for (i = opts->queue[s_p].j - 1; i >= 0; i--)
                    {
                        char one_sms[600] = { 0 };
                        char string_whithout_quotes[500] = { 0 };
                        int ia = 0, ib = 0;
                        for (ib = 0; ib < 500; ib++)
                        {
                            if (opts->queue[s_p].text[i][ib] == '\0')
                            {
                                string_whithout_quotes[ia] = opts->queue[s_p].text[i][ib];
                                break;
                            }
                            else if (opts->queue[s_p].text[i][ib] == '\"')
                            {
                                string_whithout_quotes[ia] = '&';
                                ia++;
                                string_whithout_quotes[ia] = 'r';
                                ia++;
                                string_whithout_quotes[ia] = 'd';
                                ia++;
                                string_whithout_quotes[ia] = 'q';
                                ia++;
                                string_whithout_quotes[ia] = 'u';
                                ia++;
                                string_whithout_quotes[ia] = 'o';
                                ia++;
                                string_whithout_quotes[ia] = ';';
                                ia++;
                            }
                            else
                            {
                                string_whithout_quotes[ia] = opts->queue[s_p].text[i][ib];
                                ia++;
                            }
                        }
                        //printf ("i= %d\n", i);
                        sprintf(one_sms, "\"%d %s %s\"", s_p, opts->queue[s_p].phone[i], string_whithout_quotes);
                        strcat(buffer, one_sms);
                        strcat(buffer, ", ");
                    }
                }
                if (buffer[strlen(buffer) - 2] == ',') { buffer[strlen(buffer) - 2] = '\0'; }
                strcat(buffer, "],\n\"deleted\":[\0");
                for (s_p = 7; s_p >= 0; s_p--)
                {
                    for (i = opts->deleted[s_p].j - 1; i >= 0; i--)
                    {
                        char one_sms[600] = { 0 };
                        char string_whithout_quotes[500] = { 0 };
                        int ia = 0, ib = 0;
                        for (ib = 0; ib < 500; ib++)
                        {
                            if (opts->deleted[s_p].text[i][ib] == '\0')
                            {
                                string_whithout_quotes[ia] = opts->deleted[s_p].text[i][ib];
                                break;
                            }
                            else if (opts->deleted[s_p].text[i][ib] == '\"')
                            {
                                string_whithout_quotes[ia] = '&';
                                ia++;
                                string_whithout_quotes[ia] = 'r';
                                ia++;
                                string_whithout_quotes[ia] = 'd';
                                ia++;
                                string_whithout_quotes[ia] = 'q';
                                ia++;
                                string_whithout_quotes[ia] = 'u';
                                ia++;
                                string_whithout_quotes[ia] = 'o';
                                ia++;
                                string_whithout_quotes[ia] = ';';
                                ia++;
                            }
                            else
                            {
                                string_whithout_quotes[ia] = opts->deleted[s_p].text[i][ib];
                                ia++;
                            }
                        }
                        //printf ("i= %d\n", i);
                        sprintf(one_sms, "\"%d %s %s\"", s_p, opts->deleted[s_p].phone[i], string_whithout_quotes);
                        strcat(buffer, one_sms);
                        strcat(buffer, ", ");
                    }
                }
                if (buffer[strlen(buffer) - 2] == ',') { buffer[strlen(buffer) - 2] = '\0'; }
                strcat(buffer, "],\n\"recved\":[\0");
                for (s_p = 7; s_p >= 0; s_p--)
                {
                    for (i = opts->recved[s_p].j - 1; i >= 0; i--)
                    {
                        char one_sms[600] = { 0 };
                        char string_whithout_quotes[500] = { 0 };
                        int ia = 0, ib = 0;
                        for (ib = 0; ib < 500; ib++)
                        {
                            if (opts->recved[s_p].text[i][ib] == '\0')
                            {
                                string_whithout_quotes[ia] = opts->recved[s_p].text[i][ib];
                                break;
                            }
                            else if (opts->recved[s_p].text[i][ib] == '\"')
                            {
                                string_whithout_quotes[ia] = '&';
                                ia++;
                                string_whithout_quotes[ia] = 'r';
                                ia++;
                                string_whithout_quotes[ia] = 'd';
                                ia++;
                                string_whithout_quotes[ia] = 'q';
                                ia++;
                                string_whithout_quotes[ia] = 'u';
                                ia++;
                                string_whithout_quotes[ia] = 'o';
                                ia++;
                                string_whithout_quotes[ia] = ';';
                                ia++;
                            }
                            else
                            {
                                string_whithout_quotes[ia] = opts->recved[s_p].text[i][ib];
                                ia++;
                            }
                        }
                        //printf ("i= %d\n", i);
                        sprintf(one_sms, "\"%d %s %s\"", s_p, opts->recved[s_p].phone[i], string_whithout_quotes);
                        strcat(buffer, one_sms);
                        strcat(buffer, ", ");
                    }
                }
                if (buffer[strlen(buffer) - 2] == ',') { buffer[strlen(buffer) - 2] = '\0'; }
                strcat(buffer, "]} \n");
                send(new_tcp_socket, buffer, strlen(buffer), 0);
                #ifdef WEBPOSTGETINCONSOLE
                printf("SMS to WEB: OK\n");
                #endif
            }
            else {
                //sprintf(fileadrr, "/data/www/Server/", fileadrr);
                sFile = fopen(fileadrr, "r");
                if (sFile == NULL) {
                    printf("File open: Error\n");
                    sprintf(fileadrr, "/data/www/Server/404.html", fileadrr);
                    sFile = fopen(fileadrr, "r");
                }
                else {
                    #ifdef WEBPOSTGETINCONSOLE
                    printf("File open: OK\n");
                    #endif
                }
                fseek(sFile, 0, SEEK_END);                                    //Открываем файл и перемещаем каретку в конечное положение
                nFileLen = ftell(sFile);                                       //Получаем текущее значение указателя
                fseek(sFile, 0, SEEK_SET);                                    //Перемещаем каретку в начало, чтобы корректно работать с файлом
                for (i = 0; (rc = getc(sFile)) != EOF && i < nFileLen; buffer[i++] = rc);    //Посимвольно считываем все биты из файла пока они не закончатся или не переполнится буффер
                buffer[i] = '\0';
                send(new_tcp_socket, buffer, nFileLen, 0);                                   //Выдаем буфер браузеру
                // Закрываем файл
                #ifdef WEBPOSTGETINCONSOLE
                printf("Закрытие файла: ");
                #endif
                if (fclose(sFile) == EOF)
                {
                    #ifdef WEBPOSTGETINCONSOLE
                    printf("ошибка\n");
                    #endif
                } else {
                    #ifdef WEBPOSTGETINCONSOLE
                    printf("выполнено\n");
                    #endif
                }
            }
            close(new_tcp_socket);
            end = time(NULL);
            #ifdef WEBPOSTGETINCONSOLE
            printf("\033[91mCLOSE:\033[0m OK, %f seconds from start\n", difftime(end, start));
            #endif
        }
        #ifdef WEBPOSTGETINCONSOLE
        printf("-------------------------\n");
        #endif
    }

    close(server_fd);
    printf("\033[91mSERVER CLOSED:\033[0m OK \n");
    printf("-------------------------\n");
}

void die(char* s)
{
    perror(s);
    exit(1);
}

// static void* UDP_server (void *arg)
// {
//         struct sockaddr_in si_other;
//         int udp_fd, i, slen=sizeof(si_other);
//         //char buf[BUFLEN];
//         char message[BUFLEN];
//
//         if ( (udp_fd=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
//         {
//                 die("socket");
//         }
//
//         memset((char *) &si_other, 0, sizeof(si_other));
//         si_other.sin_family = AF_INET;
//         si_other.sin_port = htons(PORT);
//
//         if (inet_aton(SERVER, &si_other.sin_addr) == 0)
//         {
//                 fprintf(stderr, "inet_aton() failed\n");
//                 exit(1);
//         }
//
//         sprintf (message, "**UPD IS WORK\0");
//
//         while(1)
//         {
//                 printf("Enter message : %s\n", message);
//                 //send the message
//                 if (sendto(udp_fd, message, strlen(message), 0, (struct sockaddr *) &si_other, slen)==-1)
//                 {
//                         die("sendto()");
//                 }
//                 sleep(1);
//                 //receive a reply and print it
//                 //clear the buffer byUFLEN, 0, (struct sockaddr *) &si_other, &slen) == -1)
//                 //{
//                 //	die("recvfrom()");
//                 //}
//         }
//
//         close(udp_fd);
//         return 0;
// }

// static void* HeartBit (void *arg)
// {
//         bool flag = false;
//         while (1)
//         {
//                 nwy_gpio_set_val(NWY_GPIO_76, flag);
//                 flag = !flag;
//         }
// }


static int do_send_sms(char* number, int encoding, int length, char* context, int async, void* opts_sms, int sms_priority)
{
    options_t* opts = (options_t*)opts_sms;
    int result = 0;
    /*
       char phone_num[NWY_SMS_MAX_ADDR_LENGTH];
       uint32_t msg_context_len;
       char msg_context[NWY_SMS_MAX_MO_MSG_LENGTH + 1];
       nwy_sms_msg_format_type_t msg_format;
     */

    nwy_sms_info_type_t sms_data = { 0 };
    //strcat(number, "\0");
    strcpy(sms_data.phone_num, number);
    sms_data.msg_context_len = length;

    if (encoding == 1) {
        sms_data.msg_format = NWY_SMS_MSG_FORMAT_TEXT_UTF8;
    }
    else if (encoding == 2) {
        sms_data.msg_format = NWY_SMS_MSG_FORMAT_TEXT_GBK;
        length = strlen(context);
    }
    else if (encoding == 0)
        sms_data.msg_format = NWY_SMS_MSG_FORMAT_TEXT_ASCII;
    else if (encoding == 3)
        sms_data.msg_format = NWY_SMS_MSG_FORMAT_BIN;


    memcpy(sms_data.msg_context, context, length + 1);

    //int nwy_sms_send_message ( nwy_sms_info_type_t *p_sms_data );

    result = nwy_sms_send_message(&sms_data);
    if (result != 0)
    {
        if (queue_flag == false) {
            sprintf(web_log, "%s[%s] %s", web_log, opts->up_time_string, "SMS_send failed\\n");
            // memcpy(opts->queue[sms_priority].phone[opts->queue[sms_priority].j], sms_data.phone_num, 12);
            // memcpy(opts->queue[sms_priority].text[opts->queue[sms_priority].j], sms_data.msg_context, 500);
        }
        else {
            printf("Try: Fail\n");
        }
        // ТУТ ДОБАВЛЯЕТСЯ В ОЧЕРЕДЬ
        memcpy(opts->queue[sms_priority].phone[opts->queue[sms_priority].j], number, 12);
        memcpy(opts->queue[sms_priority].text[opts->queue[sms_priority].j], sms_data.msg_context, 500);
        if (opts->queue[sms_priority].j <= 20) {
            opts->queue[sms_priority].j++;
        }
        else {
            int i;
            for (i = 0; i <= opts->queue[sms_priority].j - 1; i++) {
                memcpy(opts->queue[sms_priority].phone[i], opts->queue[sms_priority].phone[i + 1], 12);
                memcpy(opts->queue[sms_priority].text[i], opts->queue[sms_priority].text[i + 1], 500);
            }
        }
    }
    else {
        if (queue_flag == false) {
            sprintf(web_log, "%s[%s] %s", web_log, opts->up_time_string, "SMS_send ok\\n");
        }
        else {
            printf("Try: OK\n");
        }
        //ТУТ ДОБАВЛЯЕТСЯ В ОТПРАВЛЕННЫЕ
        if (opts->sended[sms_priority].j <= 20)
        {
            memcpy(opts->sended[sms_priority].phone[opts->sended[sms_priority].j], number, 12);
            memcpy(opts->sended[sms_priority].text[opts->sended[sms_priority].j], sms_data.msg_context, 500);
            opts->sended[sms_priority].j++;
        }
        else
        {

        }
    }
    printf("%d", result);
    //}
    return result;
}

static void test_sms_evt_handler(nwy_mt_sms_event_t ind_type, void* ind_struct)
{
    static int c = 0;

    printf("Nwy_mt_sms_event %x\n", ind_type);
    switch (ind_type) {
    case NWY_SMS_PP_IND:
    {
        nwy_sms_pp_info_type_t* sms_pp;
        sms_pp = ind_struct;
        printf("recv msg from %s\n", sms_pp->source_phone_num);
        memcpy(recv_phone, sms_pp->source_phone_num, 12);
        printf("recv msg type %d\n", sms_pp->msg_format);
        if (sms_pp->concat_sms_total > 0) {
            printf("this is the part %d of long message has %d message entry\n",
                sms_pp->concat_sms_cur, sms_pp->concat_sms_total);
            printf("concat msg id %d\n", sms_pp->concat_sms_id);
        }
        if (sms_pp->msg_format == NWY_SMS_MSG_FORMAT_TEXT_ASCII)
            printf("recv msg text %s\n", sms_pp->msg_content);
        else {
            int i = 0;
            printf("recv msg data:\n", sms_pp->msg_content);
            for (i = 0; i < sms_pp->msg_content_len; i++) {
                printf("%02x", sms_pp->msg_content[i]);
            }
            printf("\n");;
        }
        memcpy(recv_text, sms_pp->msg_content, 500);
        if (sms_pp->context_decode_type == NWY_SMS_ENCODING_GBK) {

            printf("gbk data is :\n%s\n", sms_pp->msg_decoded_content);
        }
        recv_flag = true;
        printf("FLAG IS %d\nNUM IS %s\nTEXT IS %s\n", recv_flag, recv_phone, recv_text);
        c++;
    }
    break;
    case NWY_SMS_SEND_IND:
    {
        nwy_sms_send_ind_t* send_result = ind_struct;
        if (send_result->result == 0) {
            printf("we send msg to %s result ok\n", send_result->phone_num);
        }
        else {
            printf("we send msg to %s result failed\n", send_result->phone_num);
        }
    }
    break;
    default:
        printf("we do not support this event now %x\n", ind_type);
    }
    //printf("have recv %d sms entry\n");
    printf("--------------------------------------\n");
    fflush(stdout);
}
static void del_sms_num(char* but_type, int num, void* web_opts)
{
    int i; int s_p; int sum = 0;
    options_t* opts = (options_t*)web_opts;
    if (!strcmp(but_type, "S_del_but\0"))
    {
        sum = 0;
        for (s_p = 7; s_p >= 0; s_p--)
        {
            if (opts->sended[s_p].j != 0)
            {
                sum = opts->sended[s_p].j;
                // printf("%d < %d ?", num, sum);
                if (num < sum)
                {
                    // printf("Da!\n");
                    break;
                }
                num -= opts->sended[s_p].j;
                // printf("%d \n", num);
            }
        }
        num = opts->sended[s_p].j - num - 1;
        // printf("Pr = %d, num: %d\n", s_p, num);
        for (i = 0; i <= opts->sended[s_p].j - 1; i++)
        {
            // printf("%d , %s\n", num, opts->sended[s_p].text[i]);
            // printf("i:%d, j:%d\n", i, opts->sended[s_p].j);
            num--;
            if (num < 0)
            {
                if (num == -1)
                {
                    // printf ("DEL NOW! \n");
                    memcpy(opts->deleted[s_p].text[opts->deleted[s_p].j], opts->sended[s_p].text[i], 500);
                    memcpy(opts->deleted[s_p].phone[opts->deleted[s_p].j], opts->sended[s_p].phone[i], 12);
                    if (opts->deleted[s_p].j <= 20) {
                        opts->deleted[s_p].j++;
                    }
                    else {
                        int z;
                        for (z = 0; z <= opts->deleted[s_p].j - 1; z++) {
                            memcpy(opts->deleted[s_p].phone[z], opts->deleted[s_p].phone[z + 1], 12);
                            memcpy(opts->deleted[s_p].text[z], opts->deleted[s_p].text[z + 1], 500);
                        }
                    }
                }
                // printf("%s -> %s\n", opts->sended[s_p].text[i], opts->sended[s_p].text[i+1]);
                memcpy(opts->sended[s_p].text[i], opts->sended[s_p].text[i + 1], 500);
                memcpy(opts->sended[s_p].phone[i], opts->sended[s_p].phone[i + 1], 12);
            }
        }
        opts->sended[s_p].j--;
    }
    else if (!strcmp(but_type, "Q_del_but\0")) {
        sum = 0;
        for (s_p = 7; s_p >= 0; s_p--)
        {
            if (opts->queue[s_p].j != 0)
            {
                sum = opts->queue[s_p].j;
                // printf("%d < %d ?", num, sum);
                if (num < sum)
                {
                    // printf("Da!\n");
                    break;
                }
                num -= opts->queue[s_p].j;
                // printf("%d \n", num);
            }
        }
        num = opts->queue[s_p].j - num - 1;
        // printf("Pr = %d, num: %d\n", s_p, num);
        for (i = 0; i <= opts->queue[s_p].j - 1; i++)
        {
            // printf("%d , %s\n", num, opts->queue[s_p].text[i]);
            // printf("i:%d, j:%d\n", i, opts->queue[s_p].j);
            num--;
            if (num < 0)
            {
                if (num == -1)
                {
                    // printf ("DEL NOW! \n");
                    memcpy(opts->deleted[s_p].text[opts->deleted[s_p].j], opts->queue[s_p].text[i], 500);
                    memcpy(opts->deleted[s_p].phone[opts->deleted[s_p].j], opts->queue[s_p].phone[i], 12);
                    if (opts->deleted[s_p].j <= 20) {
                        printf("%d++\n", opts->deleted[s_p].j); opts->deleted[s_p].j++;
                    }
                    else {
                        int z;
                        for (z = 0; z <= opts->deleted[s_p].j - 1; z++) {
                            memcpy(opts->deleted[s_p].phone[z], opts->deleted[s_p].phone[z + 1], 12);
                            memcpy(opts->deleted[s_p].text[z], opts->deleted[s_p].text[z + 1], 500);
                        }
                    }
                }
                // printf("%s -> %s\n", opts->queue[s_p].text[i], opts->queue[s_p].text[i+1]);
                memcpy(opts->queue[s_p].text[i], opts->queue[s_p].text[i + 1], 500);
                memcpy(opts->queue[s_p].phone[i], opts->queue[s_p].phone[i + 1], 12);
            }
        }
        opts->queue[s_p].j--;

    }
    else if (!strcmp(but_type, "R_del_but\0")) {
        sum = 0;
        for (s_p = 7; s_p >= 0; s_p--)
        {
            if (opts->recved[s_p].j != 0)
            {
                sum = opts->recved[s_p].j;
                // printf("%d < %d ?", num, sum);
                if (num < sum)
                {
                    // printf("Da!\n");
                    break;
                }
                num -= opts->recved[s_p].j;
                // printf("%d \n", num);
            }
        }
        num = opts->recved[s_p].j - num - 1;
        // printf("Pr = %d, num: %d\n", s_p, num);
        for (i = 0; i <= opts->recved[s_p].j - 1; i++)
        {
            // printf("%d , %s\n", num, opts->recved[s_p].text[i]);
            // printf("i:%d, j:%d\n", i, opts->recved[s_p].j);
            num--;
            if (num < 0)
            {
                if (num == -1)
                {
                    // printf ("DEL NOW! \n");
                    memcpy(opts->deleted[s_p].text[opts->deleted[s_p].j], opts->recved[s_p].text[i], 500);
                    memcpy(opts->deleted[s_p].phone[opts->deleted[s_p].j], opts->recved[s_p].phone[i], 12);
                    if (opts->deleted[s_p].j <= 20) {
                        opts->deleted[s_p].j++;
                    }
                    else {
                        int z;
                        for (z = 0; z <= opts->deleted[s_p].j - 1; z++) {
                            memcpy(opts->deleted[s_p].phone[z], opts->deleted[s_p].phone[z + 1], 12);
                            memcpy(opts->deleted[s_p].text[z], opts->deleted[s_p].text[z + 1], 500);
                        }
                    }
                }
                // printf("%s -> %s\n", opts->recved[s_p].text[i], opts->recved[s_p].text[i+1]);
                memcpy(opts->recved[s_p].text[i], opts->recved[s_p].text[i + 1], 500);
                memcpy(opts->recved[s_p].phone[i], opts->recved[s_p].phone[i + 1], 12);
            }
        }
        opts->recved[s_p].j--;
    }
    else if (!strcmp(but_type, "D_del_but\0")) {
        sum = 0;
        for (s_p = 7; s_p >= 0; s_p--)
        {
            if (opts->deleted[s_p].j != 0)
            {
                sum = opts->deleted[s_p].j;
                // printf("%d < %d ?", num, sum);
                if (num < sum)
                {
                    // printf("Da!\n");
                    break;
                }
                num -= opts->deleted[s_p].j;
                // printf("%d \n", num);
            }
        }
        num = opts->deleted[s_p].j - num - 1;
        // printf("Pr = %d, num: %d\n", s_p, num);
        for (i = 0; i <= opts->deleted[s_p].j - 1; i++)
        {
            // printf("%d , %s\n", num, opts->deleted[s_p].text[i]);
            // printf("i:%d, j:%d\n", i, opts->deleted[s_p].j);
            num--;
            if (num < 0)
            {
                if (num == -1)
                {
                    // printf ("DEL NOW! \n");
                }
                // printf("%s -> %s\n", opts->deleted[s_p].text[i], opts->deleted[s_p].text[i+1]);
                memcpy(opts->deleted[s_p].text[i], opts->deleted[s_p].text[i + 1], 500);
                memcpy(opts->deleted[s_p].phone[i], opts->deleted[s_p].phone[i + 1], 12);
            }
        }
        opts->deleted[s_p].j--;
    }
}

static void* try_send_sms_from_queue(void* web_opts) {
    options_t* opts = (options_t*)web_opts;
    int s_p, i;
    char phone[13];
    char text[500];
    printf("TRY TO SEND : OK\n");
    while (1)
    {
        for (s_p = 7; s_p >= 0; s_p--)
        {
            if (opts->queue[s_p].j > 0)
            {
                printf("-------------------------\n");
                queue_flag = true;
                printf("Priority: %d\n", s_p);
                printf("Try to send:\n%s\n%s\n", opts->queue[s_p].phone[0], opts->queue[s_p].text[0]);

                sprintf(phone, "%s\0", opts->queue[s_p].phone[0]);
                sprintf(text, "%s\0", opts->queue[s_p].text[0]);
                do_send_sms(phone, 0, strlen(text), text, 0, opts, 3);
                queue_flag = false;
                for (i = 0; i <= opts->queue[s_p].j - 1; i++)
                {
                    //printf("Num %d: %s --> %s\n", i, opts->queue[s_p].text[i+1], opts->queue[s_p].text[i]);
                    //printf("Num %d: %s --> %s\n\n", i, opts->queue[s_p].phone[i+1], opts->queue[s_p].phone[i]);
                    memcpy(opts->queue[s_p].text[i], opts->queue[s_p].text[i + 1], 500);             //удалим элемент под нужным нам номером
                    memcpy(opts->queue[s_p].phone[i], opts->queue[s_p].phone[i + 1], 12);
                    //printf("Num %d: %s\n", i, opts->queue[s_p].text[i]);
                    //printf("Num %d: %s\n\n", i, opts->queue[s_p].phone[i]);
                }
                opts->queue[s_p].j--;
                sleep(2);
                printf("-------------------------\n");
            }
        }
    }
}

int Write_smsTo_txt(const char* file, void* web_opts) {
    FILE* fp;
    char buff[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7 * 4];               //buffer для записи всех смс в файл
    memset(buff, 0, MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7 * 4);
    char sended_sms[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7] = { 0 };
    char recved_sms[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7] = { 0 };
    char queue_sms[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7] = { 0 };
    char deleted_sms[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7] = { 0 };
    int i, j, s_p;
    options_t* opts = (options_t*)web_opts;

    printf("Saving sms memory to %s\n", file);

    for (s_p = 7; s_p >= 0; s_p--)
    {
        for (i = 0; i <= opts->sended[s_p].j - 1; i++) {
            sprintf(sended_sms, "%s%d %s %s\r\n", sended_sms, s_p, opts->sended[s_p].phone[i], opts->sended[s_p].text[i]);
        }
        for (i = 0; i <= opts->recved[s_p].j - 1; i++) {
            sprintf(recved_sms, "%s%d %s %s\r\n", recved_sms, s_p, opts->recved[s_p].phone[i], opts->recved[s_p].text[i]);
        }
        for (i = 0; i <= opts->queue[s_p].j - 1; i++) {
            sprintf(queue_sms, "%s%d %s %s\r\n", queue_sms, s_p, opts->queue[s_p].phone[i], opts->queue[s_p].text[i]);
        }
        for (i = 0; i <= opts->deleted[s_p].j - 1; i++) {
            sprintf(deleted_sms, "%s%d %s %s\r\n", deleted_sms, s_p, opts->deleted[s_p].phone[i], opts->deleted[s_p].text[i]);
        }
    }
    sprintf(buff, "Sended\r\n%sRecved\r\n%sQueue\r\n%sDeleted\r\n%s\0", sended_sms, recved_sms, queue_sms, deleted_sms);
    //printf("%s", buff); //отладка
    fp = fopen("sms_memory.txt", "wb");
    if (NULL == fp) {
        //perror("fopen()");
        printf("fd = NULL\n");
        //return -1;
    }
    /*Тут запись в файл fwrite*/
    //sprintf(buff, "%s\0", "Sended\r\n3 +79108441072 Some sms text one\r\n3 +79108441072 Some sms text two\r\n5 +79108441072 Some sms text three\r\n3 +79108441072 Some sms text four\r\n7 +79108441072 Some sms text Five\r\nQueue\r\nRecved\r\nDeleted\r\n6 +79108441072 Some sms text six\r\n3 +79108441072 Some sms text Seven");
    fwrite(&buff, 1, sizeof(buff), fp);
    //fflush(fp);
    fclose(fp);

    /* Copy modified data back */
    //memcpy(opts, &_opts, sizeof(*opts));

    return 0;
}

int Read_smsFrom_txt(const char* file, void* web_opts) {
    FILE* fp;
    char str[1024];
    char buff[500];
    char* s, * e;
    int i, line, s_p, j;
    options_t* opts = (options_t*)web_opts;
    int flag = 0;    //Определяет в какие переменные складываем строку
    printf("Loading configuration from %s: ", file);

    /* First, copy contents of passed options into the temporary variable */
    //memcpy(&_opts, opts, sizeof(_opts));
    /* Now open and parse configuration file */
    fp = fopen("sms_memory.txt", "r");
    if (NULL == fp) {
        fp = fopen("sms_memory.txt", "wb");
        fclose(fp);
        printf("sms_memory.txt was create\n");
        fp = fopen("sms_memory.txt", "r");

    }
    line = 1;
    while (!feof(fp)) {
        memset(str, sizeof(str), 0);
        fgets(str, sizeof(str) - 1, fp);

        /* Search for comment and cut it off */
        s = strchr(str, '#');
        if (NULL != s) {
            *s = '\0';
        }

        if (0 == strlen(str)) {
            continue;
        }

        /* Skip leading spaces */
        for (s = str; isspace(*s); s++);

        /* Skip trailing spaces */
        for (i = strlen(s) - 1; i > 0 && isspace(s[i]); i--);
        s[i + 1] = '\0';

        if (0 != strlen(s)) {
            //printf("%d %s\n", line, s);
            /*parse this string*/
            if (!strcmp("Sended\0", s)) {
                flag = 1;
            }
            else if (!strcmp("Queue\0", s)) {
                flag = 2;
            }
            else if (!strcmp("Recved\0", s)) {
                flag = 3;
            }
            else if (!strcmp("Deleted\0", s)) {
                flag = 4;
            }
            else {
                i = 0; j = 0;
                while (s[i] != ' ') {
                    buff[j] = s[i];
                    i++; j++;
                }
                buff[j] = '\0';
                j = 0; i++;
                s_p = atoi(buff);
                while (s[i] != ' ') {
                    buff[j] = s[i];
                    i++; j++;
                }
                buff[j] = '\0';
                j = 0; i++;
                //printf ("flag:%d\r\n", flag);
                switch (flag) {
                case 1:
                    sprintf(opts->sended[s_p].phone[opts->sended[s_p].j], buff);
                    break;
                case 2:
                    sprintf(opts->queue[s_p].phone[opts->queue[s_p].j], buff);
                    break;
                case 3:
                    sprintf(opts->recved[s_p].phone[opts->recved[s_p].j], buff);
                    break;
                case 4:
                    sprintf(opts->deleted[s_p].phone[opts->deleted[s_p].j], buff);
                    break;
                }
                while (s[i] != '\0') {
                    buff[j] = s[i];
                    i++; j++;
                }
                buff[j] = '\0';
                switch (flag) {
                case 1:
                    sprintf(opts->sended[s_p].text[opts->sended[s_p].j], buff);
                    printf("Parse! %d\r\n%s\r\n%s\r\n\r\n", s_p, opts->sended[s_p].phone[opts->sended[s_p].j], opts->sended[s_p].text[opts->sended[s_p].j]);
                    opts->sended[s_p].j++;
                    break;
                case 2:
                    sprintf(opts->queue[s_p].text[opts->queue[s_p].j], buff);
                    printf("Parse! %d\r\n%s\r\n%s\r\n\r\n", s_p, opts->queue[s_p].phone[opts->queue[s_p].j], opts->queue[s_p].text[opts->queue[s_p].j]);
                    opts->queue[s_p].j++;
                    break;
                case 3:
                    sprintf(opts->recved[s_p].text[opts->recved[s_p].j], buff);
                    printf("Parse! %d\r\n%s\r\n%s\r\n\r\n", s_p, opts->recved[s_p].phone[opts->recved[s_p].j], opts->recved[s_p].text[opts->recved[s_p].j]);
                    opts->recved[s_p].j++;
                    break;
                case 4:
                    sprintf(opts->deleted[s_p].text[opts->deleted[s_p].j], buff);
                    printf("Parse! %d\r\n%s\r\n%s\r\n\r\n", s_p, opts->deleted[s_p].phone[opts->deleted[s_p].j], opts->deleted[s_p].text[opts->deleted[s_p].j]);
                    opts->deleted[s_p].j++;
                    break;
                }
                j = 0; i = 0;

            }
            line++;
        }
    }
    printf("OK\n");
    fclose(fp);

    /* Copy modified data back */
    //memcpy(opts, &_opts, sizeof(*opts));

    return 0;
}


// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------
// --------------------------Говнокод!-----------------------


static char*
get_config_name(int argc, char* argv[])
{
    int i;
    for (i = 1; i < argc; ++i) {
        if (0 == strcmp("-c", argv[i]) && i < argc - 1 && '-' != argv[i + 1][0]) {
            return argv[i + 1];
        }
        if (0 == strncmp("--config=", argv[i], 9)) {
            return &argv[i][9];
        }
    }
    return CONFIG_FILE;
}


int
main(int argc, char* argv[])
{
    options_t opts;
    thread_main_fn threads[] = {
        NULL, /* placeholder for agps_thread_main */
        //modem_thread_main,
        uart_read_thread_main,
        uart_write_thread_main,
        network_thread_main,
        tcp_web_thread_main,
        watchdog_thread_main,
        try_send_sms_from_queue
    };

    options_init(&opts);

    /* Scan command line to find -c file or --config=file option
     */
    load_config(get_config_name(argc, argv), &opts);

    /* Command line arguments overrides configuration file values
     */
    parse_command_line(&opts, argc, argv);

    if (0 != app_init(&opts)) {
        options_cleanup(&opts);
        return EXIT_FAILURE;
    }

    if (opts.gps_enabled) {
        threads[0] = agps_thread_main;
    }

    if (opts.gprs_enabled) {
        establish_data_connection(&opts);
    }

    if (opts.go_daemon) {
        printf("Becoming daemon\n");
        daemonize(&opts, 0);
    }

    if (0 == threads_start(&opts, threads, sizeof(threads) / sizeof(threads[0]))) {
        printf("Running\n");
        threads_wait_complete();
    }
    else {
        options_cleanup(&opts);
        return EXIT_FAILURE;
    }

    options_cleanup(&opts);
    return EXIT_SUCCESS;
}

