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
#include <locale.h>

#include "nwy_loc.h"
#include "nwy_uart.h"
#include "nwy_error.h"
#include "nwy_common.h"
#include "nwy_gpio.h"   //для хартбита
#include "nwy_pm.h"     //для хз чего - power management (*Олексий)
#include "nwy_sms.h"    //для отправки смс
#include "nwy_at.h"
#include "nwy_sim.h"

#include "config.h"
#include "configfile.h"
#include "main.h"
#include "nmea.h"
#include "data.h"
#include "at_commands.h"
#include "thread_funcs.h"
#include "uart.h"
#include "sms.h"
#include "send_at.h"
#include "modem.h"
#include "utils.h"
#include "watchdog.h"
#include "upvs.h"
#include "actions.h"

char web_log[256000];
int wb_i = 0;
//#include "tcp_web.h"   пока не добавлено
//#define WEBPOSTGETINCONSOLE  убрать если отвлекает вывод пост гет запросов в консоли
inline void d_log(const char *fmt, ...)
{
    //#ifdef WEBPOSTGETINCONSOLE
        va_list ar;
        va_start(ar, fmt);
        vprintf(fmt, ar);
        va_end(ar);
    //#endif
}

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

    //d_log("Long: %f, Lat: %f, Alt: %f, Speed: %f, Flags: 0x%08x\n", loc->longitude, loc->latitude, loc->altitude, loc->speed, loc->flags);

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

    d_log("%s", _sendbuf);

    pthread_cond_signal(&msg_ready); // wake up UART_WRITE thread
}

options_t *_opts = NULL;


static void
nwy_loc_event_handler(nwy_loc_ind_t* msg)
{
    static nmea_msg_t nmsg;
    nmea_err_t err;
    int i;

    switch (msg->ind_type) {
    case NWY_LOC_LOCATION_INFO_IND_MSG:
        d_log("NWY_LOC_LOCATION_INFO_IND_MSG\n");
        process_msg_location(&msg->ind_msg.gps_loc_msg);
        break;
#if 1
    case NWY_LOC_STATUS_INFO_IND_MSG:
        d_log("NWY_LOC_STATUS_INFO_IND_MSG\n");
        break;

    case NWY_LOC_SV_INFO_IND_MSG:
        if (msg->ind_msg.sv_info_msg.size > 0) {
            d_log("NWY_LOC_SV_INFO_IND_MSG (%d)\n\n", msg->ind_msg.sv_info_msg.size);
            for (i = 0; i < msg->ind_msg.sv_info_msg.size; ++i) {
                nwy_gps_sv_info_t* info = &msg->ind_msg.sv_info_msg.sv_list[i];
                d_log("\tAzimuth: %f\n", info->azimuth);
                d_log("\tElevation: %f\n", info->elevation);
                d_log("\tPRN: %d\n", info->prn);
                d_log("\tSNR: %f\n\n", info->snr);
            }
        }
        break;

    case NWY_LOC_NMEA_INFO_IND_MSG:
        printf("%s\n", msg->ind_msg.nmea_msg.nmea);
        err = nmea_parse(msg->ind_msg.nmea_msg.nmea, &nmsg);
        if (NMEA_ERR_OK == err) {
            if (NMEA_GSV == nmsg.type) {
                if (nmsg.gsv.complete) {
                    //d_log("%s: %d satellites\n", nmea_system_name(nmsg.type), nmsg.gsv.count);
                }
            }
            else if (NMEA_GGA == nmsg.type) {
                strcpy(_opts->nmea_gga, msg->ind_msg.nmea_msg.nmea);
            }
            else if (NMEA_GSA == nmsg.type) {
                strcpy(_opts->nmea_gsa, msg->ind_msg.nmea_msg.nmea);
            }
            else if (NMEA_RMC == nmsg.type) {
                strcpy(_opts->nmea_rmc, msg->ind_msg.nmea_msg.nmea);
            }
        }
        else {
            d_log("nmea_parse(\"%s\") = %d\n", msg->ind_msg.nmea_msg.nmea, err);
        }
        break;

    case NWY_LOC_CAPABILITIES_INFO_IND_MSG:
        d_log("NWY_LOC_CAPABILITIES_INFO_IND_MSG\n");
        break;

    case NWY_LOC_UTC_TIME_REQ_IND_MSG:
        d_log("NWY_LOC_UTC_TIME_REQ_IND_MSG\n");
        break;

    case NWY_LOC_XTRA_DATA_REQ_IND_MSG:
        d_log("NWY_LOC_XTRA_DATA_REQ_IND_MSG\n");
        break;

    case NWY_LOC_AGPS_STATUS_IND_MSG:
        d_log("NWY_LOC_AGPS_STATUS_IND_MSG\n");
        break;

    case NWY_LOC_NI_NOTIFICATION_IND_MSG:
        d_log("NWY_LOC_NI_NOTIFICATION_IND_MSG\n");
        break;

    case NWY_LOC_XTRA_REPORT_SERVER_IND_MSG:
        d_log("NWY_LOC_XTRA_REPORT_SERVER_IND_MSG\n");
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

    d_log("Init: AGPS (v6)\n");

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
        d_log("nwy_loc_set_indication_mask() failed, error code = %d\n", result);
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

    d_log("Init: Network\n");
    res = get_UUID(mac);
    if (res < 0) {
        d_log("get_UUID() failed. code=%d\n", res);
        return res;
    }

    //sprintf(opts->uuid, "NEOWAY_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    sprintf(opts->uuid, "NEOWAY_CACAB");
    d_log("UUID %s\n", opts->uuid);

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
        d_log("Cannot resolve host name %s\n", opts->broadcast_addr);
        return -2;
    }

    memset(&opts->baddr, 0, sizeof(opts->baddr));
    opts->baddr.sin_family = AF_INET;
    opts->baddr.sin_addr.s_addr = *((in_addr_t*)hent->h_addr_list[0]);
    opts->baddr.sin_port = htons(broadcast_port);

    d_log("Broadcast address is %s:%u\n", inet_ntoa(opts->baddr.sin_addr), broadcast_port);

    opts->tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (opts->tcp_sock < 0) {
        perror("socket(TCP)");
        return -1;
    }

    hent = gethostbyname(opts->kud_address);
    if (NULL == hent) {
        d_log("Cannot resolve host name %s\n", opts->broadcast_addr);
        return -2;
    }

    memset(&kud, 0, sizeof(kud));
    kud.sin_family = AF_INET;
    kud.sin_addr.s_addr = *((in_addr_t*)hent->h_addr_list[0]);
    kud.sin_port = htons(broadcast_port);

    d_log("Gateway address is %s:%u\n", inet_ntoa(kud.sin_addr), broadcast_port);

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
    d_log("AGPS thread start\n");
    nwy_loc_location_type_t loc_info;

    for (;;) {
        int result = nwy_loc_get_curr_location(&loc_info, 10);
        if (0 != result) {
            if (-703 == result) {
                d_log("nwy_loc_get_curr_location() timeout\n");
            }
            else {
                d_log("-----\nnwy_loc_get_curr_location() failed, returned value was %d\n", result);
                d_log("Trying to recover\n-----\n");
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

    //d_log("\0x1B[33mBroadcasting announce\0x1B[37m\n");
    #ifdef WEBPOSTGETINCONSOLE
    d_log("%s ----------- Broadcasting announce --------------\n", strtime);
    #endif

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
            d_log("Other side closed connection\n");
        }
        d_log("Received %d bytes\n", res);
    }
#endif

    if (FD_ISSET(opts->udp_broadcast, fds)) {
        sin.sin_family = AF_INET;
        sin.sin_addr.s_addr = opts->baddr.sin_addr.s_addr;
        sin.sin_port = htons(19001);
        res = recvfrom(opts->udp_broadcast, buf, sizeof(buf), 0, (struct sockaddr*) & sin, &sinsize);
        if (res < 0)
            perror("recvfrom()");

        d_log("Datagram received (%d bytes) from %s\n", res, inet_ntoa(sin.sin_addr));
    }

    if (res >= 0) {
        buf[res] = 0;
        d_log("%s\n", buf);
    }
}


char*
get_file_contents(const char* fn)
{
    FILE* fp = fopen(fn, "rb");
    char* data;
    long flen;

    if (NULL == fp) {
        d_log("%s: ", fn);
        perror("fopen()");
        return strdup("");
    }

    fseek(fp, 0, SEEK_END);
    flen = ftell(fp);

    d_log("Reading %s (%ld bytes)\n", fn, flen);

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


char* xmls[XML_MAX];


static void XMLCALL
process_element_start(void* userdata, const XML_Char* name, const XML_Char** attrs)
{
    xml_context_t *ctx = (xml_context_t *)userdata;
    xml_tag_t *tag;

    tag = xml_tag(ctx, name, NULL);
    if(NULL != tag) {
        if(NULL == ctx->last) {
            ctx->last = tag;
        } else {
            if(ctx->tag_closed) {
                ctx->last->next = tag;
                tag->parent = ctx->last->parent;
            } else {
                ctx->last->child = tag;
                tag->parent = ctx->last;
            }
        }

        if(NULL == ctx->root) {
            ctx->root = tag;
        }
    }

    ++ctx->level;
    ctx->tag_closed = FALSE;
}


static void XMLCALL
process_element_end(void *userdata, const XML_Char *name)
{
    xml_context_t *ctx = (xml_context_t*)userdata;

    (void)name;

    if (--ctx->level == 0) {
        //process_get(opts);
    }

    ctx->tag_closed = TRUE;
}


static void
process_uvps_request(xml_tag_t *root)
{
    if(xml_is(root, "body")) {

    }
}


static void XMLCALL
process_character_data(void *userdata, const XML_Char *text, int len)
{
    xml_context_t *ctx = (xml_context_t *)userdata;
    //ctx->last->content = buf_strncpy(ctx->strings, text, len);
}


void
read_xmls()
{
    xmls[XML_INFO] = get_file_contents("/data/xml/info.xml");
    xmls[XML_SMS] = get_file_contents("/data/xml/sms.xml");
    xmls[XML_GPRS] = get_file_contents("/data/xml/gprs.xml");
    xmls[XML_CONFIG] = get_file_contents("/data/xml/config.xml");
    xmls[XML_GPSGLONASS] = get_file_contents("/data/xml/gpsglonass.xml");
    xmls[XML_WEB] = get_file_contents("/data/xml/web.xml");
    xmls[XML_QUERY_STATE_VARIABLE] = get_file_contents("/data/xml/queryvariable.xml");
}


static void
queryStateVariable(options_t* opts)
{
    char value[256] = {0};
    if (0 == strcasecmp(opts->xml_variable, "telno")) {
        sprintf(value, "<value>%s</value>", opts->phone_number);

    } else if (0 == strcasecmp(opts->xml_variable, "connected")) {
        sprintf(value, "<value>%s</value>", opts->gprs_enabled ? "Да" : "Нет");

    } else if (0 == strcasecmp(opts->xml_variable, "text")) {
        sprintf(value, "<value>%s</value>", opts->sms_text);

    } else if (0 == strcasecmp(opts->xml_variable, "www")) {
        sprintf(value, "<value>%s</value>", "http://10.7.6.1");

    } else if (0 == strcasecmp(opts->xml_variable, "ip")) {
        sprintf(value, "<value>%s</value>", opts->gsm_ip_state);

    } else if (0 == strcasecmp(opts->xml_variable, "pin")) {
        sprintf(value, "<value>%s</value>", opts->pin);

    } else if (0 == strcasecmp(opts->xml_variable, "router")) {
        sprintf(value, "<value>%s</value>", opts->gsm_ip_state);

    } else if (0 == strcasecmp(opts->xml_variable, "sigLevel")) {
        sprintf(value, "<value>%s</value>", "siglevel");

    } else if (0 == strcasecmp(opts->xml_variable, "operator")) {
        sprintf(value, "<value>Beeline</value>");

    } else if (0 == strcasecmp(opts->xml_variable, "localtime")) {
        sprintf(value, "<value>LocalTime</value>");

    } else if (0 == strcasecmp(opts->xml_variable, "CURRENT_XYZ")) {
        sprintf(value,  "<struct>"
                            "<latitude><value>%.2f</value></latitude>"
                            "<longitude><value>%.2f</value></longitude>"
                            "<altitude><value>%.2f</value></altitude>"
                        "</struct>",
                            opts->last_nmea_msg.rmc.latitude,
                            opts->last_nmea_msg.rmc.longitude,
                            opts->level);

    } else if (0 == strcasecmp(opts->xml_variable, "NMEA_DATA")) {
        sprintf(value, "<struct>"
                            "<GGA><value>%s</value></GGA>",
                            "<GSA><value>%s</value></GSA>",
                            "<RMC><value>%s</value></RMC>",
                        "</struct>", opts->nmea_gga, opts->nmea_gsa, opts->nmea_rmc);

    } else if (0 == strcasecmp(opts->xml_variable, "TotalOdometer")) {
        sprintf(value, "<value>%.1f</value>", opts->total_mileage);
    } else if (0 == strcasecmp(opts->xml_variable, "CurrentOdometer")) {
        sprintf(value, "<value>%.1f</value>", opts->mileage);
    }
    else {
        sprintf(value, "%s", "Неизвестный параметр");
    }
    _sendbuflen = sprintf(_sendbuf, xmls[XML_QUERY_STATE_VARIABLE], opts->r_uuid, opts->xml_variable, value, opts->xml_variable);
    _sendbuf[_sendbuflen] = 0;
    d_log(_sendbuf);
}


static void
process_get(options_t* opts)
{
    char* p = strchr(opts->r_uuid, ':');

    switch (opts->elem) {
        case XML_BODY:
            if (XML_CMD_QUERY_STATE_VARIABLE == opts->xml_cmd) {
                queryStateVariable(opts);
            }
            return;
        default:
            break;
    }

    if(NULL != p) {
        ++p;
        if (0 == strcasecmp(p, "1")) {
            _sendbuflen = sprintf(_sendbuf, xmls[XML_SMS], opts->r_uuid);
        }
        else if (0 == strcasecmp(p, "2")) {
            _sendbuflen = sprintf(_sendbuf, xmls[XML_GPRS], opts->r_uuid);
        }
        else if (0 == strcasecmp(p, "3")) {
            _sendbuflen = sprintf(_sendbuf, xmls[XML_CONFIG], opts->r_uuid);
        }
        else if(0 == strcasecmp(p, "4")) {
            _sendbuflen = sprintf(_sendbuf, xmls[XML_GPSGLONASS], opts->r_uuid);
        }
    } else {
        _sendbuflen = sprintf(_sendbuf, xmls[XML_INFO], opts->r_uuid);
    }
}


static int
action_send_sms(options_t* opts)
{
    int webs;
    struct hostent* hent;
    struct sockaddr_in addr;
    struct in_addr* localhost;
    char req[1024];
    int reqlen;

    d_log("Sending SMS to \"%s\"\n", opts->phone_number);
    d_log("Text: %s\n", opts->sms_text);

    reqlen = snprintf(req, sizeof(req),
        "POST / HTTP/1.1\r\n"
        "Connection: close\r\n"
        "Content-type: application/x-www-form-urlencoded\r\n"
        "\r\n"
        "SMS_send\n"
        "%s %s\r\n", opts->phone_number, opts->sms_text
    );
    wb_i += snprintf(&web_log[wb_i], sizeof(web_log), "[%s]%s %s %s", opts->up_time_string, "KDUSV SEND:\\n", opts->phone_number, opts->sms_text);
    hent = gethostbyname("localhost");
    if (NULL == hent) {
        herror("gethostbyname()");
        return;
    }

    localhost = (struct in_addr*)hent->h_addr_list[0];

    d_log("IP=%s\n", inet_ntoa(*localhost));

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = localhost->s_addr;
    addr.sin_port = htons(80);

    webs = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (webs < 0) {
        perror("socket()");
        return -1;
    }

    d_log("Connecting...\n");
    if (connect(webs, (struct sockaddr*) & addr, sizeof(addr)) < 0) {
        perror("connect()");
        return -1;
    }

    d_log("Sending...\n");
    if (send(webs, req, reqlen, 0) < 0) {
        perror("send()");
        return -1;
    }

#if 0
    d_log("Receiving...\n");
    int len = recv(webs, req, sizeof(req), 0);
    if (len < 0) {
        perror("recv()");
        return -1;
    }
    else {
        req[len] = 0;
        d_log("RECV: %s\n", req);
    }
#endif

    d_log("Closing socket...\n");

    shutdown(webs, SHUT_RDWR);
    close(webs);
    _sendbuflen = sprintf(_sendbuf, "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
        "<body urn=\"%s\">"
            "<action>"
                "<name>%s</name>"
            "</action>"
        "</body>",
        opts->uuid,
        opts->xml_action
    );

    d_log("Finished sending SMS\n");

    _sendbuf[_sendbuflen] = 0;
    printf("%s\n",_sendbuf);
    return 0;
}


static void XMLCALL
process_character_data_old(void* userdata, const XML_Char* text, int len)
{
    options_t* opts = (options_t*)userdata;
    char temp[1024];

    switch (opts->elem) {
    case XML_UUID:
        strncpy(opts->r_uuid, text, len);
        opts->r_uuid[len] = 0;
        #ifdef WEBPOSTGETINCONSOLE
        d_log("UUID = %s\n", opts->r_uuid);
        #endif
        break;
    case XML_NAME:
        strncpy(opts->xml_action_name, text, len);
        opts->xml_action_name[len] = 0;
        //which_action(opts, opts->xml_action_name);
        #ifdef WEBPOSTGETINCONSOLE
        d_log("Action name = %s\n", temp);
        #endif
        memset(opts->phone_number, 0, sizeof(opts->phone_number));
        memset(opts->sms_text, 0, sizeof(opts->sms_text));
        opts->xml_action = action_send_sms;
        break;
    case XML_VALUE:
        strncpy(temp, text, len);
        temp[len] = 0;
        #ifdef WEBPOSTGETINCONSOLE
        d_log("SET %s = %s\n", opts->xml_variable, temp);
        #endif
        if (0 == strcasecmp("callNumber", opts->xml_variable)) {
            strcpy(opts->phone_number, temp);
            #ifdef WEBPOSTGETINCONSOLE
            d_log("telno %s\n", opts->phone_number);
            #endif
        }
        else if (0 == strcasecmp("data", opts->xml_variable)) {
            strcpy(opts->sms_text, temp);
            #ifdef WEBPOSTGETINCONSOLE
            d_log("text %s\n", opts->sms_text);
            #endif
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
process_element_start_old(void* userdata, const XML_Char* name, const XML_Char** attrs)
{
    options_t *opts = (options_t *)userdata;
    int i;

    if (0 == opts->level) {
        opts->xml_cmd = XML_CMD_NONE;
    }

    if (1 == opts->level) {
        if (0 == strcasecmp(name, "queryStateVariable")) {
            opts->xml_cmd = XML_CMD_QUERY_STATE_VARIABLE;
        }
        else if (0 == strcasecmp(name, "action")) {
            opts->xml_cmd = XML_CMD_ACTION;
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
            }
            else if (0 == strcasecmp(name, "argumentList")) {
                opts->xml_cmd = XML_CMD_ACTION_ARGS;
                d_log("argumentList\n");
            }
            break;
        }
    }

    //if (3 == opts->level) {
    if (XML_CMD_ACTION_ARGS == opts->xml_cmd) {
        if(0 != strcasecmp(name, "struct")) {
            memset(opts->xml_variable, 0, sizeof(opts->xml_variable));
            strcpy(opts->xml_variable, name);
            d_log("Arg: %s\n", name);
            opts->elem = XML_VALUE;
        }
    }
    //}

    if (XML_CMD_ACTION_ARGS == opts->xml_cmd && 0 == strcasecmp("value", name)) {
        opts->elem = XML_VALUE;
    }
    else {
        opts->elem = XML_NONE;
    }

    d_log("%d %s\n", opts->level, name);

    if (0 == strcasecmp(name, "get")) {
        d_log("XML_GET\n");
        opts->elem = XML_GET;
    }
    else if (0 == strcasecmp(name, "UUID")) {
        opts->elem = XML_UUID;
        d_log("XML_UUID\n");
    }
    else if (0 == strcasecmp(name, "body")) {
        opts->elem = XML_BODY;
        d_log("XML_BODY\n");
        if (NULL != attrs) {
            for (i = 0; NULL != attrs[i]; i += 2) {
                if (0 == strcasecmp("urn", attrs[i])) {
                    memset(opts->r_uuid, 0, sizeof(opts->r_uuid));
                    strcpy(opts->r_uuid, attrs[i + 1]);
                    d_log("R_UUID = %s\n", attrs[i + 1]);
                }
            }
        }
    }

    ++opts->level;
}


static void XMLCALL
process_element_end_old(void* userdata, const XML_Char* name)
{
    options_t* opts = (options_t*)userdata;

    (void)name;

    if (--opts->level == 0) {
        process_get(opts);
    }
}


static void *
network_thread_main(void* arg)
{
    options_t* opts = (options_t*)arg;
    fd_set rfds, wfds;
    struct timeval tm;
    int res, maxsock;
    time_t last_ts, now;
    int i, sock;
    buf_t tags, chars;
    char buf[8192];
    XML_Parser parser;
    xml_context_t ctx;

    parser = XML_ParserCreate(NULL);
    buf_init(&tags, sizeof(xml_tag_t)*100);
    buf_init(&chars, 8192);
    xml_context_init(&ctx, &tags, &chars);
    printf("------------------------");
    d_log("Network thread start\n");

    read_xmls();

    last_ts = time(NULL);
    for (;;) {
        memset(_sendbuf, 0, sizeof(_sendbuf));
        _sendbuflen = 0;

        FD_ZERO(&rfds);
        FD_ZERO(&wfds);

        FD_SET(opts->udp_broadcast, &wfds);
        FD_SET(opts->tcp_sock, &rfds);

        maxsock = MAX(opts->udp_broadcast, opts->tcp_sock);

        tm.tv_sec = opts->broadcast_period;
        tm.tv_usec = 0;

        res = select(maxsock+1, &rfds, &wfds, NULL, &tm);
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
                //#ifdef WEBPOSTGETINCONSOLE
                d_log("Accepted connection from %s:%u\n", inet_ntoa(in_addr.sin_addr), ntohs(in_addr.sin_port));
                //#endif

                res = recv(sock, buf, sizeof(buf), 0);
                if (res > 0) {
                    memset(_sendbuf, 0, sizeof(_sendbuf));
                    _sendbuflen = 0;

                    d_log("%s", buf);
                    opts->level = 0;
                    opts->xml_action = NULL;

                    xml_context_reset(&ctx);

                    XML_SetUserData(parser, opts);
                    XML_SetStartElementHandler(parser, process_element_start_old);
                    XML_SetEndElementHandler(parser, process_element_end_old);
                    XML_SetCharacterDataHandler(parser, process_character_data_old);
                    XML_Parse(parser, buf, res, TRUE);

                    process_get(opts);

                    if (NULL != opts->xml_action) {
                        opts->xml_action(opts);
                    }

                    d_log("%s", _sendbuf);
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


static int
detect_sim_card(options_t *opts)
{
    char *res, *reply;
    int rc;

    printf("Detecting SIM card.\n");

    res = NULL;
    reply = NULL;
    pthread_mutex_lock(&opts->mutex_modem);
    rc = nwy_at_send_cmd("AT+CCID", &res, &reply);
    pthread_mutex_unlock(&opts->mutex_modem);

    printf("res=\"%s\", reply=\"%s\"\n", res==NULL? "": res, reply==NULL? "": reply);

    if(0 == rc && NULL != reply) {
        if(0 == strcasecmp(reply, "ERROR")) {
            opts->sim_status = NWY_SIM_NOT_INSERTED;
            opts->gprs_enabled = FALSE;
            printf("SIM card not found. GSM services disabled.\n");
        } else {
            opts->sim_status = NWY_SIM_READY;
            printf("SIM card ready. %s\n", res);
        }
    } else {
        printf("nwy_at_send_cmd() returns error code %d\n", rc);
    }

    free(res);
    free(reply);

    return 0;
}


static char *_pid_file;


static void
remove_pid(void)
{
    unlink(_pid_file); // But do not close the file! It will maintain on disk until closed!
}


static void
write_pid(options_t *opts)
{
    FILE *pf;

    printf("Saving PID to %s\n", opts->pid_file);
    pf = fopen(opts->pid_file, "w");
    if(NULL != pf) {
        fprintf(pf, "%d\n", getpid());
        fclose(pf);
    } else {
        printf("Cannot open PID file %s for write\n", opts->pid_file);
    }

    _pid_file = opts->pid_file; // for atexit
    atexit(remove_pid);
}


int
app_init(options_t* opts)
{
    int res;
    char buf[32] = {0};

    if (0 != network_init(opts)) {
        d_log("Network init failed\n");
        exit(EXIT_FAILURE);
    }

    opts->uart_fd = uart_init(opts->uart_tty, opts->uart_baud_rate);
    if (-1 == opts->uart_fd) {
        d_log("UART init failed\n");
        exit(EXIT_FAILURE);
    }

    opts->modem_fd = modem_init(opts->modem_tty, opts->uart_baud_rate);
    if (-1 == opts->modem_fd) {
        printf("MODEM init failed\n");
        exit(EXIT_FAILURE);
    }

    if (opts->gprs_enabled) {
        /* Get SIM status */
        /*d_log("Querying SIM (SLOT 1) status\n");
        opts->sim_status = nwy_sim_get_card_status(NWY_SIM_ID_SLOT_1);

        if (opts->sim_status < 0) {
            d_log("Error (%d) querying SIM (SLOT 1) status. GPRS service disabled.\n", opts->sim_status);
            printf("Error (%d) querying SIM (SLOT 1) status. GPRS service disabled.\n", opts->sim_status);
            opts->sim_status = NWY_SIM_NOT_INSERTED;
            opts->gprs_enabled = FALSE;
        } else {
            switch (opts->sim_status) {
            case NWY_SIM_NOT_INSERTED:
                d_log("SIM card not present (SLOT 1). Disabling SMS and GPRS services.\n");
                printf("SIM card not present (SLOT 1). Disabling SMS and GPRS services.\n");
                opts->gprs_enabled = FALSE;
                break;

                case NWY_SIM_PIN_REQ:
                    res = nwy_sim_verify_pin(NWY_SIM_ID_SLOT_1, opts->pin);
                    if(res < 0) {
                        d_log("SIM PIN verify failed. Error code is %d\n", res);
                        printf("SIM PIN verify failed. Error code is %d\n", res);
                        opts->gprs_enabled = FALSE;
                    };
                    break;

                case NWY_SIM_READY:
                    d_log("SIM card installed and ready\n");
                    printf("SIM card installed and ready\n");
                    break;
            }
        }*/
        detect_sim_card(opts);
    }

    /*opts->modem_fd = modem_init(opts->modem_tty, opts->uart_baud_rate);
    if (-1 == opts->modem_fd) {
        d_log("MODEM init failed\n");
        exit(EXIT_FAILURE);
    }*/

#if 0
    if (opts->gps_enabled) {
        if (0 != agps_init()) {
            d_log("AGPS init failed\n");
            //exit(EXIT_FAILURE);
        }
    }
#endif

    nwy_gpio_set_dir(NWY_GPIO_78, NWY_OUTPUT);
    nwy_gpio_set_val(NWY_GPIO_78, NWY_HIGH);

    write_pid(opts);

    return 0;
}


// --------------------------Alexander_code-----------------------
// --------------------------Alexander_code-----------------------
// --------------------------Alexander_code-----------------------


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

#define BUFLEN 512             //Max length of buffer

char buffer[128768] = { 0 };    //Буфер для вычитывания файлов и принятия внешних запросов
//Переменные, для вычитывания данных из внешнего запроса
char method[10],               //Метод Post, get или что то еще
fileadrr[100],                 //Адрес файла к которому обращение
filetype[10],                  //Расширение файла
postcomand[20],                //Команда для сервера через POST
postbody[1000];                //Тело Post запроса
char fullfileadrr[100] = { 0 };
//Переменные, для хранения основных параметров монитора ищи в options_t в utils.h

char phone_num[13], sms_text[500] = { '\0' }; //Переменные для работы с смс
char recv_phone[13], recv_text[500] = { '\0' };
bool recv_flag = false;
bool queue_flag = false;

int opt = 1, rc;
int user_id = 0;                   //2 - Admin, 1 - Viewer, 0 - Her kakoi-to
time_t start, end;
int server_fd, new_tcp_socket, valread;  //

struct sockaddr_in address;
int addrlen = sizeof(address);

int GPRS_state_now = 0;           //0 - Не установлено соединение 1 - соединение установлено, получен ip

void Header_Parse();                                //Функция которая разбирает ключевые заголовки из buffer и складывает в предназначенные для этого массивы
void Start_Socket();                                //Socket+bind+listen
int check_GPRS_state(void* web_opts);
static void del_sms_num(char* but_type, int num, void* web_opts);
static void* HeartBit(void* arg);
static int do_send_sms(char* number, int encoding, int length, char* context, int async, void* opts_sms, int sms_priority);
static void test_sms_evt_handler(nwy_mt_sms_event_t ind_type, void* ind_struct);

void Start_Socket()
{
    //Создание дескриптера сокета
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    start = time(NULL);
    d_log("SOCKET: OK\n");
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
    d_log("BIND: OK\n");
    if (listen(server_fd, 10) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    d_log("LISTEN: OK\n");
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
        method[j] = buffer[i];                           //POST or GET
        i++;
        j++;
    }
    method[j] = '\0';
    i++; j = 0; i++;
    while (buffer[i] != ' ')                            //чтение имени запрашиваемого файла вида index.html
    {
        if (buffer[i] == '\0') break;
        fileadrr[j] = buffer[i];
        i++;
        j++;
        if (buffer[i] == '.')                           //Если в запрсе точка
        {
            fileadrr[j] = '\0';                         //закончили читать имя файла
            i++; j = 0;
            while (buffer[i] != ' ')                    //читаем расширение файла
            {
                filetype[j] = buffer[i];                //
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
        while ((buffer[i] != '\n') || (buffer[i - 2] != '\n'))    //ищет конец первой строки \r\n\r\n
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

static void* tcp_web_thread_main(void* arg)
{
    setlocale (LC_ALL,""); // установить используемую системой локаль
    options_t* opts = (options_t*)arg;
    d_log("WEB DIR: %s\n",opts->web_dir_i_path);
    char* at_com = malloc(sizeof(char) * 100);
    int i = 0;
    int j = 0;
    FILE* sFile;                            //Открытие файлов
    long int nFileLen;                      //Сюда пишем позицию
    signal(SIGPIPE, SIG_IGN);                //Игнорим ситуацию если пакет послан, но не был принят
    Start_Socket();                         //Запуск Socket+bind+listen
    At_init(opts);                          //Инициализация для работы с AT

    end = time(NULL);
    int _uptime = (int)difftime(end, start);
    printf ("%d",_uptime);

    char sss[5], mmm[5], hhh[5];            //складываем в строки секунды часы и минуты
    (_uptime % 60 < 10) ? snprintf(sss, sizeof(sss), "0%d", _uptime % 60) : snprintf(sss, sizeof(sss), "%d", _uptime % 60);
    ((_uptime / 60) % 60 < 10) ? snprintf(mmm, sizeof(mmm), "0%d", (_uptime / 60) % 60) : snprintf(mmm, sizeof(mmm), "%d", (_uptime / 60) % 60);
    ((_uptime / 3600) % 60 < 10) ? snprintf(hhh, sizeof(hhh), "0%d", (_uptime / 3600) % 60) : snprintf(hhh, sizeof(hhh), "%d", (_uptime / 3600) % 60);
    snprintf(opts->up_time_string, sizeof(opts->up_time_string), "%s : %s : %s\0", hhh, mmm, sss);

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
    {       d_log ("i = %d\n", i);
            //int j = opts->sended[s_p].j;
            snprintf(opts->sended[3].phone[i], "%s\0", "+7910123456"); d_log("%s\n", opts->sended[3].phone[i]); snprintf(opts->sended[3].text[i], "%s%d\0", "Text", i); d_log("%s\n", opts->sended[3].text[i]);
    }
    opts->sended[7].j = 4;
    //Тестовое заполнение массива сообщений
    for (i = opts->sended[7].j-1; i >= 0; i--)
    {       d_log ("i = %d\n", i);
            //int j = opts->sended[s_p].j;
            snprintf(opts->sended[7].phone[i], "%s\0", "+7910123456"); d_log("%s\n", opts->sended[7].phone[i]); snprintf(opts->sended[7].text[i], "%s%d\0", "Text", i); d_log("%s\n", opts->sended[7].text[i]);
    }
    opts->deleted[5].j = 5;
    for (i = opts->deleted[5].j-1; i >= 0; i--)
    {       d_log ("i = %d\n", i);
            //int j = opts->sended[s_p].j;
            snprintf(opts->deleted[5].phone[i], "%s\0", "+7910123456"); d_log("%s\n", opts->deleted[5].phone[i]); snprintf(opts->deleted[5].text[i], "%s%d\0", "Text", i); d_log("%s\n", opts->deleted[5].text[i]);
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
            d_log("\033[91mACCEPT:\033[0m FAIL, %f seconds from start\n", difftime(end, start));
#endif
            continue;                                                                        //exit(EXIT_FAILURE);
        }
        end = time(NULL);
#ifdef WEBPOSTGETINCONSOLE
        d_log("\033[91mACCEPT:\033[0m OK, %f seconds from start\n", difftime(end, start));
#endif
        valread = recv(new_tcp_socket, buffer, sizeof(buffer) - 1, 0);                              //Разбираем запрос от браузера
        end = time(NULL);
#ifdef WEBPOSTGETINCONSOLE
        d_log("\033[91mRecv:\033[0m End, %f seconds from start\n", difftime(end, start));
#endif
        if (valread <= 0)
        {
            send(new_tcp_socket, "HTTP/1.1 400\r\n", strlen("HTTP/1.1 400\r\n"), 0);
            //d_log("Fatalerror\n");
            close(new_tcp_socket);
            continue;
        }
        buffer[valread] = '\0';                                                 //Добавляем конец строки на позиции конца строки
        #ifdef WEBPOSTGETINCONSOLE
        d_log("\033[91mREAD:\033[0m OK, %d symbols\n", valread);
        #endif
        Header_Parse();
        snprintf(fullfileadrr, 100, "%s.%s", fileadrr, filetype);              //Собираем целый адресс из имени и расширения
        //Вывод на экран всего что там напарсилось
        #ifdef WEBPOSTGETINCONSOLE
        //d_log("%s\n", buffer);
        d_log("\033[92mMethod:\033[0m %s\n", method);
        //d_log("\033[92mAddr:\033[0m %s \n", fileadrr);
        //d_log("\033[92mType:\033[0m %s\n", filetype);
        //d_log("\033[92mComand:\033[0m %s\n", postcomand);
        //d_log("\033[92mBody:\033[0m %s\n", postbody);
        d_log("\033[92mAddr:\033[0m %s \n", fullfileadrr);
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
                    wb_i += snprintf(&web_log[wb_i], sizeof(web_log), "[%s] %s", opts->up_time_string, "Admin is unlogged\\n");
                }
                else if (user_id == 1) {
                    wb_i += snprintf(&web_log[wb_i], sizeof(web_log), "[%s] %s", opts->up_time_string, "User is unlogged\\n");
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
                d_log("Num: %s\n SMS: %s\n Len: %d\n", phone_num, sms_text, strlen(sms_text));
                do_send_sms(phone_num, 0, strlen(sms_text), sms_text, 0, opts, 3);
            }
            else if (!strcmp(postcomand, "SMS_save\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                Write_smsTo_txt("sms_memory.txt\0", opts);
                #ifdef WEBPOSTGETINCONSOLE
                d_log("SMS_saving: OK\n");
                #endif
            }
            else if (!strcmp(postcomand, "save_netstat\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                int i = 0, j = 0;
                char inet_web[16];          //ifconfig bridge0 inet 192.168.0.0 netmask 255.255.255.0
                char netmask_web[16];
                char gps_time_web[5];
                char dhcp = 'V';
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
                d_log("inet: %s\nnetmask: %s\ngps_time: %s\n", inet_web, netmask_web, gps_time_web);
                char sys_com[100] = { 0 };
                snprintf(sys_com, 100, "ifconfig bridge0 inet %s netmask %s\n", inet_web, netmask_web);
                FILE* fp;
                char sysconf[500] = { 0 };
                snprintf(sysconf, sizeof(sysconf), "%s\n%s\n%s\n%c\n", inet_web, netmask_web, gps_time_web, dhcp);
                char fullpath[30];
                strcat(fullpath, opts->web_dir_i_path);   //собираем адрсс из PATH в neowayhelper.conf
                strcat(fullpath, "/");                    //слэша
                strcat(fullpath, "sysconf.txt");               //и имени файла
                printf("Fopen: %s\n", fullpath);
                fp = fopen(fullpath, "wb");
                if (NULL == fp) {
                    //perror("fopen()");
                    printf("fd = NULL\n");
                    //return -1;
                }
                fwrite(&sysconf, 1, sizeof(sysconf), fp);
                fclose(fp);
                //Write_sysconfto_txt("sysconf.txt\0", opts);
                system(sys_com);
                wb_i += snprintf(&web_log[wb_i], 64000, "[%s] Конфигурация сохранена!\\nip:%s\\nnetmask:%s\\n", opts->up_time_string, inet_web, netmask_web);
                //d_log("%s", sys_com);
                // j = 0; i++;
            }
            else if (!strcmp(postcomand, "del_sms\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                int i = 0, j = 0;
                char but_selected[20];
                char but_num[10];
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
                d_log("Button: %s\nBut_Num: %s\nInt_But: %d\n", but_selected, but_num, atoi(but_num));
                del_sms_num(but_selected, atoi(but_num), opts);
                //Write_smsTo_txt("sms_memory.txt\0", opts);
            }
            /*
            * FLAG UP TO SEND UART COMMAND to STM
            */
            else if (!strcmp(postcomand, "Mileage_reset\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                pthread_mutex_lock(&opts->mutex);
                opts->reset_mileage = 1;
                pthread_mutex_unlock(&opts->mutex);
            }
            /*
            * Reboot Neoway
            */
            else if (!strcmp(postcomand, "Reboot\0"))
            {
                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                system("reboot\n");
            }
            /*
            * Unknown command
            */
            else
            {
                send(new_tcp_socket, "HTTP/1.1 400 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
            }
            close(new_tcp_socket);
            end = time(NULL);
            #ifdef WEBPOSTGETINCONSOLE
            d_log("\033[91mCLOSE:\033[0m OK, %f seconds from start\n", difftime(end, start));
            #endif
        }

        if (method[0] == 'G' && method[1] == 'E' && method[2] == 'T')
        {
            switch (user_id)
            {
            case 0:
                if (!strcmp(fullfileadrr, "index.html\0") || !strcmp(filetype, "ico\0") || !strcmp(filetype, "png\0") || !strcmp(fullfileadrr, "blocked.html\0") || !strcmp(fullfileadrr, "login.js\0") || !strcmp(fullfileadrr, "picnic.css\0") || !strcmp(fullfileadrr, "sms_memory.txt"))
                {
                    d_log("Guest Ok\n");
                }
                else
                {
                    snprintf(fullfileadrr, 100, "404.html");
                    snprintf(filetype, 10, "html");
                    #ifdef WEBPOSTGETINCONSOLE
                    d_log("Guest sheet %s\n", fullfileadrr);
                    #endif
                }
                break;
            case 1:
                if (!strcmp(fullfileadrr, "index.html\0"))
                {
                    snprintf(fullfileadrr, 100, "indexWebViewer.html");
                    wb_i += snprintf(&web_log[wb_i], 64000, "[%s] %s", opts->up_time_string, "User is logged\\n");
                }
                #ifdef WEBPOSTGETINCONSOLE
                d_log("Hallo User :)\n");
                #endif
                break;
            case 2:
                if (!strcmp(fullfileadrr, "index.html\0"))
                {
                    snprintf(fullfileadrr, 100, "indexWeb.html");
                    wb_i += snprintf(&web_log[wb_i], 64000, "[%s] %s", opts->up_time_string, "Admin is logged\\n");
                }
                #ifdef WEBPOSTGETINCONSOLE
                d_log("Hallo Admin :)\n");
                #endif
                break;
            }


            if (send(new_tcp_socket, "HTTP/1.1 200 OK\r\n", strlen("HTTP/1.1 200 OK\r\n"), 0) == -1)
            {
                perror("ERROR SEND\n");
                //printf("ERROR SEND\n");
            }
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
            if (!strcmp(fullfileadrr, "data.json\0"))
            {
                //printf("Start sending at\n");
                send_at_cmd("AT+CSQ\0", opts);
                send_at_cmd("AT+CIMI\0", opts);
                send_at_cmd("AT+CGSN\0", opts);
                send_at_cmd("AT$MYGPSPOS=1\0", opts);
                send_at_cmd("AT$MYGPSPOS=3\0", opts);
                send_at_cmd("AT$MYSYSINFO\0", opts);
                //printf("End sending at\n");
                end = time(NULL);

                int _uptime = (int)difftime(end, start);
                printf ("%d\n",_uptime);
                if(_uptime > 1000000000) //If date 1970 -> 2021
                {
                    start = time(NULL);
                    _uptime = (int)difftime(end, start);
                }
                char sss[5], mmm[5], hhh[5];            //складываем в строки секунды часы и минуты
                (_uptime % 60 < 10) ? snprintf(sss, sizeof(sss), "0%d", _uptime % 60) : snprintf(sss, sizeof(sss), "%d", _uptime % 60);
                ((_uptime / 60) % 60 < 10) ? snprintf(mmm, sizeof(mmm), "0%d", (_uptime / 60) % 60) : snprintf(mmm, sizeof(mmm), "%d", (_uptime / 60) % 60);
                ((_uptime / 3600) % 60 < 10) ? snprintf(hhh, sizeof(hhh), "0%d", (_uptime / 3600) % 60) : snprintf(hhh, sizeof(hhh), "%d", (_uptime / 3600) % 60);

                /*(((int)difftime(end, start) % 60 < 10) ? snprintf(sss, sizeof(sss), "0%d", (int)difftime(end, start) % 60) : snprintf(sss, sizeof(sss), "%d", (int)difftime(end, start) % 60));
                (((int)(difftime(end, start) / 60) % 60 < 10) ? snprintf(mmm, sizeof(mmm), "0%d", (int)(difftime(end, start) / 60) % 60) : snprintf(mmm, sizeof(mmm), "%d", (int)(difftime(end, start) / 60) % 60));
                (((int)(difftime(end, start) / 3600) % 60 < 10) ? snprintf(hhh, sizeof(hhh), "0%d", (int)(difftime(end, start) / 3600) % 60) : snprintf(hhh, sizeof(hhh), "%d", (int)(difftime(end, start) / 3600) % 60));
                */snprintf(opts->up_time_string, sizeof(opts->up_time_string), "%s : %s : %s\0", hhh, mmm, sss);

                char pwrtype[20];
                if(opts->power_source == POWER_SOURCE_NORMAL){
                    snprintf(pwrtype, sizeof(pwrtype), "От сети");
                } else if(opts->power_source == POWER_SOURCE_BATTERY) {
                    snprintf(pwrtype, sizeof(pwrtype), "От батареи");
                } else {
                    snprintf(pwrtype, sizeof(pwrtype), "Error");
                }

                if (!strcmp(opts->threed_fix, "invalid\0")){ //Если не достоверно соединение с , то не выводим ничего с этим связанного
                    snprintf(opts->sput_time, sizeof(opts->sput_time), "No GPS signal\0");
                }

                if (!strcmp(opts->imsi, "")){ //Если не достоверно соединение с , то не выводим ничего с этим связанного
                    snprintf(opts->imsi, sizeof(opts->imsi), "No SIM in slot\0");
                    snprintf(opts->operator_cod, sizeof(opts->operator_cod), "No reg\0");
                }

                char _oneChar;
                FILE* gsmipFile;
                gsmipFile = fopen("/var/run/gsm.connected", "r");
                if (gsmipFile == NULL) {
                    #ifdef WEBPOSTGETINCONSOLE
                    printf("/var/run/gsm.connected Error open or not found\n");
                    #endif
                    snprintf(opts->gsm_ip_state, sizeof(opts->gsm_ip_state), "Соединение не установлено");
                } else {
                    #ifdef WEBPOSTGETINCONSOLE
                    printf("/var/run/gsm.connected open: OK\n");
                    #endif
                        fgets(opts->gsm_ip_state, 126, gsmipFile);
                        int i = 0;
                        for (i = 0; i < sizeof(opts->gsm_ip_state);i++, j++)
                        {
                            if(opts->gsm_ip_state[i] == '\n')
                            {
                                opts->gsm_ip_state[i] = 0;
                            }
                        }
                        snprintf(opts->gsm_time, sizeof(opts->gsm_time), "%s", ctime(&end));
                        i = 0;
                        for (i = 0; i < sizeof(opts->gsm_time);i++)
                        {
                            if(opts->gsm_time[i] == '\n')
                            {
                                opts->gsm_time[i] = 0;
                            }
                        }
                    // Закрываем файл
                    #ifdef WEBPOSTGETINCONSOLE
                    printf("Закрытие файла /var/run/gsm.connected: ");
                    #endif
                    if (fclose(gsmipFile) == EOF)
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
                //check_GPRS_state(opts);
                /*
                * Mutex for get total_mileage & mileage
                */

                //printf("Start Waiting mutex at\n");

                pthread_mutex_lock(&opts->mutex);

                //printf("End wainting mutex\n");
                snprintf(buffer, 65000, "{\"rssi\" : \"%s\",\n"
                                        "\"threed_fix\" : \"%s\",\n"
                                        "\"gps_cords\" : \" %c %f\xC2\xB0 %c %f\xC2\xB0\",\n"
                                        "\"sys_time\" : \" %s\",\n"
                                        "\"num_sput\" : \"%d\",\n"
                                        "\"reg_in_mesh\" : \" %s %s\",\n"
                                        "\"mobile_data\" : \"%s\",\n"
                                        "\"imsi\" : \"%s\",\n"
                                        "\"imei\" : \"%s\",\n"
                                        "\"uptime\" : \"%s\",\n"
                                        "\"carrige_mileage\" : \"%f км\",\n"
                                        "\"last_mileage\" : \"%f км\",\n"
                                        "\"power_type\" : \"%s\",\n"
                                        "\"gsm_ip_state\" : \"%s\",\n"
                                        "\"speed\" : \"%f\",\n"
                                        "\"gsm_time\" : \"%s\"}\0",
                                        opts->rssi,
                                        opts->threed_fix,
                                        opts->lat_sign, opts->lat, opts->lon_sign, opts->lon,
                                        opts->sput_time,
                                        opts->num_sput_val,
                                        opts->operator_cod, opts->country_cod,
                                        opts->mobile_data,
                                        opts->imsi,
                                        opts->imei,
                                        opts->up_time_string,
                                        opts->total_mileage,
                                        opts->mileage,
                                        pwrtype,
                                        opts->gsm_ip_state,
                                        opts->speed,
                                        opts->gsm_time);
                pthread_mutex_unlock(&opts->mutex);

                send(new_tcp_socket, buffer, strlen(buffer), 0);
                at_com = NULL;
            }
            else if (!strcmp(fullfileadrr, "log.json\0")) {
                snprintf(buffer, 65000, "{\"log\" : \"%s\"\n}\0", &web_log[0]);
                send(new_tcp_socket, buffer, strlen(buffer), 0);
            }
            else if (!strcmp(fullfileadrr, "sms.json\0")) {
                /*count_to_save_sms++;
                if (count_to_save_sms > 30)
                {
                    count_to_save_sms = 0;
                    Write_smsTo_txt("sms_memory.txt\0", opts);
                }*/
                if (recv_flag == true)                             //если приходило смс, добавить его в список
                {
                    recv_flag = false;
                    //d_log("FLAG IS %d\nNUM IS %s\nTEXT IS %s\n", recv_flag, recv_phone, recv_text);
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
                    // d_log("FLAG IS %d\nNUM IS %s\nTEXT IS %s\n", recv_flag, recv_phone, recv_text);
                }
                int i = 0;
                snprintf(buffer, 20, "{\"sended\":[\0");
                for (s_p = 7; s_p >= 0; s_p--)
                {
                    for (i = opts->sended[s_p].j - 1; i >= 0; i--)
                    {
                        char one_sms[600] = { 0 };
                        //d_log ("i= %d\n", i);
                        char string_whithout_quotes[500] = { 0 };
                        int ia = 0, ib = 0;
                        for (ib = 0; ib < 500; ib++)                   //Находит все кавычки и заменяет их для корректного отображения браузером
                        {                                              //Для sms.json в части sended
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
                        snprintf(one_sms, 600, "\"%d %s %s\"", s_p, opts->sended[s_p].phone[i], string_whithout_quotes);
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
                        for (ib = 0; ib < 500; ib++)                        //Находит все кавычки и заменяет их для корректного отображения браузером
                        {                                                   //в части queue
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
                        //d_log ("i= %d\n", i);
                        snprintf(one_sms, 600, "\"%d %s %s\"", s_p, opts->queue[s_p].phone[i], string_whithout_quotes);
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
                        for (ib = 0; ib < 500; ib++)                         //Находит все кавычки и заменяет их для корректного отображения браузером
                        {                                                    //в части deleted
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
                        //d_log ("i= %d\n", i);
                        snprintf(one_sms, 600, "\"%d %s %s\"", s_p, opts->deleted[s_p].phone[i], string_whithout_quotes);
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
                        for (ib = 0; ib < 500; ib++)                            //Находит все кавычки и заменяет их для корректного отображения браузером
                        {                                                       //в части recved
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
                        //d_log ("i= %d\n", i);
                        snprintf(one_sms, 600, "\"%d %s %s\"", s_p, opts->recved[s_p].phone[i], string_whithout_quotes);
                        strcat(buffer, one_sms);
                        strcat(buffer, ", ");
                    }
                }
                if (buffer[strlen(buffer) - 2] == ',') { buffer[strlen(buffer) - 2] = '\0'; }
                strcat(buffer, "]} \n");
                send(new_tcp_socket, buffer, strlen(buffer), 0);
                #ifdef WEBPOSTGETINCONSOLE
                d_log("SMS to WEB: OK\n");
                #endif
            }
            else {
                char fullpath[30] = {0};
                strcat(fullpath, opts->web_dir_i_path);   //собираем адрсс из PATH в neowayhelper.conf
                strcat(fullpath, "/");                    //слэша
                strcat(fullpath, fullfileadrr);               //и имени файла
                d_log("Fopen: %s\n", fullpath);
                sFile = fopen(fullpath, "r");
                if (sFile == NULL) {
                    d_log("File open: Error\n");
                    snprintf(fullfileadrr, 100, "/data/www/Server/404.html");  //В случае ошибки или отсутствия файла подсунуть 404
                    sFile = fopen(fullfileadrr, "r");
                }
                else {
                    #ifdef WEBPOSTGETINCONSOLE
                    d_log("File open: OK\n");
                    #endif
                }
                fseek(sFile, 0, SEEK_END);                                    //Открываем файл и перемещаем каретку в конечное положение
                nFileLen = ftell(sFile);                                      //Получаем текущее значение указателя
                fseek(sFile, 0, SEEK_SET);                                    //Перемещаем каретку в начало, чтобы корректно работать с файлом
                for (i = 0; (rc = getc(sFile)) != EOF && i < nFileLen; buffer[i++] = rc);    //Посимвольно считываем все биты из файла пока они не закончатся или не переполнится буффер
                buffer[i] = '\0';
                //printf("%d %p %d \n" , new_tcp_socket, buffer, nFileLen);
                //printf("%s\n", buffer);
                if(send(new_tcp_socket, buffer, nFileLen, 0)<0)
                {
                    perror("Send page:");
                }                                   //Выдаем буфер браузеру
                // Закрываем файл
                #ifdef WEBPOSTGETINCONSOLE
                d_log("Закрытие файла: ");
                #endif
                if (fclose(sFile) == EOF)
                {
                    #ifdef WEBPOSTGETINCONSOLE
                    d_log("ошибка\n");
                    #endif
                } else {
                    #ifdef WEBPOSTGETINCONSOLE
                    d_log("выполнено\n");
                    #endif
                }
            }
            close(new_tcp_socket);
            end = time(NULL);
            #ifdef WEBPOSTGETINCONSOLE
            d_log("\033[91mCLOSE:\033[0m OK, %f seconds from start\n", difftime(end, start));
            #endif
        }
        #ifdef WEBPOSTGETINCONSOLE
        d_log("-------------------------\n");
        #endif
    }
    close(server_fd);
    d_log("\033[91mSERVER CLOSED:\033[0m OK \n");
    d_log("-------------------------\n");
}

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
            wb_i+= snprintf(&web_log[wb_i], sizeof(web_log),"[%s] %s", opts->up_time_string, "SMS_send failed\\n");
            // memcpy(opts->queue[sms_priority].phone[opts->queue[sms_priority].j], sms_data.phone_num, 12);
            // memcpy(opts->queue[sms_priority].text[opts->queue[sms_priority].j], sms_data.msg_context, 500);
        }
        else {
            d_log("Try to send sms: Fail\n");
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
            wb_i += snprintf(&web_log[wb_i], sizeof(web_log), "[%s] %s", opts->up_time_string, "SMS_send ok\\n");
        }
        else {
            d_log("Try to send sms: OK\n");
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
    d_log("%d", result);
    //Write_smsTo_txt("sms_memory.txt\0", opts);
    //}
    return result;
}

static void test_sms_evt_handler(nwy_mt_sms_event_t ind_type, void* ind_struct)
{
    static int c = 0;

    d_log("Nwy_mt_sms_event %x\n", ind_type);
    switch (ind_type) {
    case NWY_SMS_PP_IND:
    {
        nwy_sms_pp_info_type_t* sms_pp;
        sms_pp = ind_struct;
        d_log("recv msg from %s\n", sms_pp->source_phone_num);
        memcpy(recv_phone, sms_pp->source_phone_num, 12);
        d_log("recv msg type %d\n", sms_pp->msg_format);
        if (sms_pp->concat_sms_total > 0) {
            d_log("this is the part %d of long message has %d message entry\n",
                sms_pp->concat_sms_cur, sms_pp->concat_sms_total);
            d_log("concat msg id %d\n", sms_pp->concat_sms_id);
        }
        if (sms_pp->msg_format == NWY_SMS_MSG_FORMAT_TEXT_ASCII)
            d_log("recv msg text %s\n", sms_pp->msg_content);
        else {
            int i = 0;
            d_log("recv msg data:\n", sms_pp->msg_content);
            for (i = 0; i < sms_pp->msg_content_len; i++) {
                d_log("%02x", sms_pp->msg_content[i]);
            }
            d_log("\n");;
        }
        memcpy(recv_text, sms_pp->msg_content, 500);
        if (sms_pp->context_decode_type == NWY_SMS_ENCODING_GBK) {

            d_log("gbk data is :\n%s\n", sms_pp->msg_decoded_content);
        }
        recv_flag = true;
        d_log("FLAG IS %d\nNUM IS %s\nTEXT IS %s\n", recv_flag, recv_phone, recv_text);
        c++;
    }
    break;
    case NWY_SMS_SEND_IND:
    {
        nwy_sms_send_ind_t* send_result = ind_struct;
        if (send_result->result == 0) {
            d_log("we send msg to %s result ok\n", send_result->phone_num);
        }
        else {
            d_log("we send msg to %s result failed\n", send_result->phone_num);
        }
    }
    break;
    default:
        d_log("we do not support this event now %x\n", ind_type);
    }
    //d_log("have recv %d sms entry\n");
    d_log("--------------------------------------\n");
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
                // d_log("%d < %d ?", num, sum);
                if (num < sum)
                {
                    // d_log("Da!\n");
                    break;
                }
                num -= opts->sended[s_p].j;
                // d_log("%d \n", num);
            }
        }
        num = opts->sended[s_p].j - num - 1;
        // d_log("Pr = %d, num: %d\n", s_p, num);
        for (i = 0; i <= opts->sended[s_p].j - 1; i++)
        {
            // d_log("%d , %s\n", num, opts->sended[s_p].text[i]);
            // d_log("i:%d, j:%d\n", i, opts->sended[s_p].j);
            num--;
            if (num < 0)
            {
                if (num == -1)
                {
                    // d_log ("DEL NOW! \n");
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
                // d_log("%s -> %s\n", opts->sended[s_p].text[i], opts->sended[s_p].text[i+1]);
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
                // d_log("%d < %d ?", num, sum);
                if (num < sum)
                {
                    // d_log("Da!\n");
                    break;
                }
                num -= opts->queue[s_p].j;
                // d_log("%d \n", num);
            }
        }
        num = opts->queue[s_p].j - num - 1;
        // d_log("Pr = %d, num: %d\n", s_p, num);
        for (i = 0; i <= opts->queue[s_p].j - 1; i++)
        {
            // d_log("%d , %s\n", num, opts->queue[s_p].text[i]);
            // d_log("i:%d, j:%d\n", i, opts->queue[s_p].j);
            num--;
            if (num < 0)
            {
                if (num == -1)
                {
                    // d_log ("DEL NOW! \n");
                    memcpy(opts->deleted[s_p].text[opts->deleted[s_p].j], opts->queue[s_p].text[i], 500);
                    memcpy(opts->deleted[s_p].phone[opts->deleted[s_p].j], opts->queue[s_p].phone[i], 12);
                    if (opts->deleted[s_p].j <= 20) {
                        d_log("%d++\n", opts->deleted[s_p].j); opts->deleted[s_p].j++;
                    }
                    else {
                        int z;
                        for (z = 0; z <= opts->deleted[s_p].j - 1; z++) {
                            memcpy(opts->deleted[s_p].phone[z], opts->deleted[s_p].phone[z + 1], 12);
                            memcpy(opts->deleted[s_p].text[z], opts->deleted[s_p].text[z + 1], 500);
                        }
                    }
                }
                // d_log("%s -> %s\n", opts->queue[s_p].text[i], opts->queue[s_p].text[i+1]);
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
                // d_log("%d < %d ?", num, sum);
                if (num < sum)
                {
                    // d_log("Da!\n");
                    break;
                }
                num -= opts->recved[s_p].j;
                // d_log("%d \n", num);
            }
        }
        num = opts->recved[s_p].j - num - 1;
        // d_log("Pr = %d, num: %d\n", s_p, num);
        for (i = 0; i <= opts->recved[s_p].j - 1; i++)
        {
            // d_log("%d , %s\n", num, opts->recved[s_p].text[i]);
            // d_log("i:%d, j:%d\n", i, opts->recved[s_p].j);
            num--;
            if (num < 0)
            {
                if (num == -1)
                {
                    // d_log ("DEL NOW! \n");
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
                // d_log("%s -> %s\n", opts->recved[s_p].text[i], opts->recved[s_p].text[i+1]);
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
                // d_log("%d < %d ?", num, sum);
                if (num < sum)
                {
                    // d_log("Da!\n");
                    break;
                }
                num -= opts->deleted[s_p].j;
                // d_log("%d \n", num);
            }
        }
        num = opts->deleted[s_p].j - num - 1;
        // d_log("Pr = %d, num: %d\n", s_p, num);
        for (i = 0; i <= opts->deleted[s_p].j - 1; i++)
        {
            // d_log("%d , %s\n", num, opts->deleted[s_p].text[i]);
            // d_log("i:%d, j:%d\n", i, opts->deleted[s_p].j);
            num--;
            if (num < 0)
            {
                if (num == -1)
                {
                    // d_log ("DEL NOW! \n");
                }
                // d_log("%s -> %s\n", opts->deleted[s_p].text[i], opts->deleted[s_p].text[i+1]);
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
    d_log("TRY TO SEND : OK\n");
    while (1)
    {
        for (s_p = 7; s_p >= 0; s_p--)
        {
            if (opts->queue[s_p].j > 0)
            {
                d_log("-------------------------\n");
                queue_flag = true;
                d_log("Priority: %d\n", s_p);
                d_log("Try to send:\n%s\n%s\n", opts->queue[s_p].phone[0], opts->queue[s_p].text[0]);

                snprintf(phone, 13, "%s\0", opts->queue[s_p].phone[0]);
                snprintf(text, 500, "%s\0", opts->queue[s_p].text[0]);
                do_send_sms(phone, 0, strlen(text), text, 0, opts, 3);
                queue_flag = false;
                for (i = 0; i <= opts->queue[s_p].j - 1; i++)
                {
                    //d_log("Num %d: %s --> %s\n", i, opts->queue[s_p].text[i+1], opts->queue[s_p].text[i]);
                    //d_log("Num %d: %s --> %s\n\n", i, opts->queue[s_p].phone[i+1], opts->queue[s_p].phone[i]);
                    memcpy(opts->queue[s_p].text[i], opts->queue[s_p].text[i + 1], 500);     //удалим элемент под нужным нам номером
                    memcpy(opts->queue[s_p].phone[i], opts->queue[s_p].phone[i + 1], 12);
                    //d_log("Num %d: %s\n", i, opts->queue[s_p].text[i]);
                    //d_log("Num %d: %s\n\n", i, opts->queue[s_p].phone[i]);
                }
                opts->queue[s_p].j--;
                sleep(2);
                d_log("-------------------------\n");
            }
        }
    }
}

int check_GPRS_state(void* web_opts)
{
    /*
    * Get info about gsm_ip from /var/run/gsm.connected
    */
    end = time(NULL);
    options_t* opts = (options_t*)web_opts;
    char _oneChar;
    FILE* gsmipFile;
    gsmipFile = fopen("/var/run/gsm.connected", "r");
    if (gsmipFile == NULL) {
        #ifdef WEBPOSTGETINCONSOLE
        printf("/var/run/gsm.connected Error open or not found\n");
        #endif
        snprintf(opts->gsm_ip_state, sizeof(opts->gsm_ip_state), "Соединение не установлено");
    } else {
        #ifdef WEBPOSTGETINCONSOLE
        printf("/var/run/gsm.connected open: OK\n");
        #endif
        //fseek(gsmipFile, 0, SEEK_END);                                    //Открываем файл и перемещаем каретку в конечное положение
        //int gsmipFileLen = ftell(gsmipFile);                                      //Получаем текущее значение указателя
        //fseek(gsmipFile, 0, SEEK_SET);                                    //Перемещаем каретку в начало, чтобы корректно работать с файлом
        //for (i = 0; (_oneChar = getc(gsmipFile)) != EOF && i < gsmipFileLen; opts->gsm_ip_state[i++] = rc);    //Посимвольно считываем все биты из файла пока они не закончатся или не переполнится буффер
        //opts->gsm_ip_state[i] = '\0';

            fgets(opts->gsm_ip_state, 126, gsmipFile);
            int j = 0;
            int i = 0;
            for (i = 0; i < sizeof(opts->gsm_ip_state);i++, j++)
            {
                if(opts->gsm_ip_state[i] == '\n')
                {
                    opts->gsm_ip_state[i] = 0;
                }
            }
            snprintf(opts->gsm_time, sizeof(opts->gsm_time), "%s", ctime(&end));
            i = 0;
            for (i = 0; i < sizeof(opts->gsm_time);i++)
            {
                if(opts->gsm_time[i] == '\n')
                {
                    opts->gsm_time[i] = 0;
                }
            }
        //
        //
        //

        //long int ttime;

        // Считываем текущее время
        //ttime = time (NULL);

        // С помощью функции ctime преобразуем считанное время в
        // локальное, а затем в строку и выводим в консоль.
        //printf ("Время: %s\n",ctime (&ttime) );


        // Закрываем файл
        #ifdef WEBPOSTGETINCONSOLE
        printf("Закрытие файла /var/run/gsm.connected: ");
        #endif
        if (fclose(gsmipFile) == EOF)
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
    return 0;
}
// --------------------------Alexander_code_end!-----------------------
// --------------------------Alexander_code_end!-----------------------
// --------------------------Alexander_code_end!-----------------------
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

static options_t opts;

int
main(int argc, char* argv[])
{
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
    _opts = &opts; // HACK

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
        d_log("Becoming daemon\n");
        daemonize(&opts, 0);
    }

    if (0 == threads_start(&opts, threads, sizeof(threads) / sizeof(threads[0]))) {
        d_log("Running\n");
        threads_wait_complete();
    }
    else {
        options_cleanup(&opts);
        return EXIT_FAILURE;
    }

    options_cleanup(&opts);
    return EXIT_SUCCESS;
}
