#include "utils.h"


extern int wb_i;
extern char web_log[];


extern int _sendbuflen;
extern int _sendbuf[];


static actionarg_t _args[MAX_ACTION_ARGS];
int _nargs;

static char _strings[ACTION_STRINGS_MAX];
static char* _str;


static int
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

    if (connect(webs, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("connect()");
        return -1;
    }

    if (send(webs, req, reqlen, 0) < 0) {
        perror("send()");
        return -1;
    }

    int len = recv(webs, req, sizeof(req), 0);
    if (len < 0) {
        perror("recv()");
        return -1;
    }
    else {
        req[len] = 0;
        d_log("RECV: %s\n", req);
    }

    shutdown(webs, SHUT_RDWR);
    close(webs);

    return 0;
}


int
action_get_sms(options_t* opts)
{
    return 0;
}


int
action_del_sms(options_t* opts)
{
    return 0;
}


int
action_clear_out_queue(options_t* opts)
{
    return 0;
}


int
action_clear_in_queue(options_t* opts)
{
    return 0;
}


int
action_gprs_connect(options_t* opts)
{
    return 0;
}


int
action_gprs_disconnect(options_t* opts)
{
    return 0;
}


static actiondef_t _actions[] = {
    /* SMS actions */
    { "clearOutQueue",  action_clear_out_queue },
    { "clearInQueue",   action_clear_in_queue },
    { "putSMS",         action_send_sms },
    { "getSMS",         action_get_sms },
    { "delSMS",         action_del_sms },
    { "removeSMS",      action_del_sms },
    /* GPRS actions */
    { "connect",        action_gprs_connect },
    { "disconnect",     action_gprs_disconnect },
    /* CONFIG actions */
    { "setDateTime",    NULL },
    /* GPS/GLONASS actions */
    { "setOdometerValue", NULL },
    { "resetCurrentOdometer", NULL },
    { "setPassword",    NULL },
};


char *
arg_strdup(const char* s)
{
    int len;
    char* res;

    if (NULL == s) {
        return NULL;
    }

    len = strlen(s);
    if (_str + len > _strings + ACTION_STRINGS_MAX) {
        return "(no memory)";
    }

    res = _str;
    _str += len + 1;

    strcpy(res, s);
    res[len] = 0;

    return res;
}


actionarg_t *
arg_new(const char *name) 
{
    actionarg_t* arg;

    if (_nargs == MAX_ACTION_ARGS) {
        return NULL;
    }

    arg = &_args[_narg++];

    arg->name = arg_strdup(name);
    arg->value = NULL;
    arg->child = NULL;
    arg->next = NULL;

    return arg;
}


char*
argval(options_t* opts, const char* name)
{
    actionarg_t* arg;

    for (arg = opts->args; NULL != arg; arg = arg->next) {
        if (0 == strcasecmp(name, arg->name)) {
            return arg->value;
        }
    }

    return NULL;
}


actionarg_t *
arg_find(options_t* opts, const char* name, int deep)
{
    actionarg_t* arg, res;

    for (arg = opts->args; NULL != arg; arg = arg->next) {
        if (0 == strcasecmp(name, arg->name) {
            return arg;
        }
        
        if (deep && NULL != arg->child) {
            res = arg_find(arg->child, name, deep);
            if (NULL != res) {
                return res;
            }
        }
    }

    return NULL;
}


void
actions_init()
{
    _nargs = 0;
    _str = _strings;
}


void
which_action(options_t *opts, const char* name)
{
    int i;

    opts->xml_action = NULL;
    for (i = 0; i < sizeof(_actions) / sizeof(_actions[0]); ++i) {
        if (0 == strcasecmp(name, _actions[i].name)) {
            opts->xml_action = _actions[i].handler;
            return;
        }
    }
}

