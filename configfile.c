#include <stdio.h>
#include <memory.h>
#include <ctype.h>

#include "configfile.h"


static options_t _opts;


confoption_t options[] = {
    { "uart_tty",           TYPE_STRING,    &_opts.uart_tty },
    { "baudrate",           TYPE_INT,       &_opts.baud_rate },
    { "modem_tty",          TYPE_STRING,    &_opts.modem_tty },
    { "modem_baudrate",     TYPE_INT,       &_opts.modem_baud_rate},
    { "go_daemon",          TYPE_BOOL,      &_opts.go_daemon },
    { "enable_gps",         TYPE_BOOL,      NULL },
    { "debug_print",        TYPE_BOOL,      &_opts.debug_print },
    { "gps_enabled",        TYPE_BOOL,      &_opts.gps_enabled },
    { "gprs_enabled",       TYPE_BOOL,      &_opts.gprs_enabled },
    { "broadcast_address",  TYPE_IPADDR,    &_opts.udp_broadcast },
    { "upvs_port",          TYPE_INT,       NULL }
};  


static int
parse_bool_arg(char *arg)
{
    if('\0' == *arg) { // empty argument is interpreted as TRUE
        return 1;
    }

    if(0 == strcasecmp(arg, "yes") || 0 == strcasecmp(arg, "true") || 0 == strcasecmp(arg, "1")) {
        return 1;
    }

    if(0 == strcasecmp(arg, "no") || 0 == strcasecmp(arg, "false") || 0 == strcasecmp(arg, "0")) {
        return 0;
    }

    return -1; // Error
}


static void
process_config_line(char *line, int lineno)
{
    char *s, *ptr;
    int i, arg;
    float farg;

    /* Search for the end of the keyword. Keyword separated from value by spaces or tabs 
     */
    for(s=line; !isblank(*s); ++s);
    
    *s++ = '\0';

    /* Skip blanks and advance to the beginning of the value 
     */
    while('='==*s || isblank(*s)) {
        s++;
    }

    printf("'%s'='%s'\n", line, s);

    for(i=0; i<(sizeof(options)/sizeof(options[0])); ++i) {
        if(0 == strcasecmp(options[i].keyword, line)) {
            if(NULL == options[i].argptr) {
                continue;
            }

            switch(options[i].type) {
                case TYPE_STRING:
                case TYPE_IPADDR:
                    free(*((char **)options[i].argptr));
                    *((char **)options[i].argptr) = strdup(s);
                    break;

                case TYPE_CHAR_ARRAY:
                    strcpy((char *)options[i].argptr, s); // Not error-prone, buffer overflow can occur!
                    break;

                case TYPE_BOOL:
                    arg = parse_bool_arg(s);
                    if(arg < 0) {
                        printf("Config: Parameter %s on line %d requires boolean arg, got '%s'\n", line, lineno, s);
                        arg = 0;
                    }
                    *((int *)options[i].argptr) = arg;
                    break;

                case TYPE_INT:
                    if('0'==s[0] && ('x'==s[1] || 'X'==s[1])) {
                        arg = strtol(s, &ptr, 16);
                    } else {
                        arg = strtol(s, &ptr, 10);
                    }

                    if('\0' != *ptr) {
                        printf("Config: Parameter %s on line %d: Malformed decimal value '%s'\n", line, lineno, s);
                    } else {
                        *((int *)options[i].argptr) = arg;
                    }
                    break;

                case TYPE_FLOAT:
                    farg = strtof(s, &ptr);
                    
                    if('\0' != *ptr) {
                        printf("Config: Parameter %s on line %d: Malformed float value '%s'\n", line, lineno, s);
                    } else {
                        *((float *)options[i].argptr) = farg;
                    }
                    break;

                default: // Redundant, but code standard requires that default case always be present
                    break; 
            }
            break; // for
        }
    }

    /* Unknown keywords are silently ignored 
     */
}


int
load_config(const char *file, /* in, out */ options_t *opts)
{
    FILE *fp;
    char str[1024];
    char *s, *e;
    int i, line;

    printf("Loading configuration from %s\n", file);

    /* First, copy contents of passed options into the temporary variable */
    memcpy(&_opts, opts, sizeof(_opts));

    /* Now open and parse configuration file */
    fp = fopen(file, "r");
    if(NULL == fp) {
        perror("fopen()");
        return -1;
    }

    line = 1;

    while(!feof(fp)) {
        memset(str, sizeof(str), 0);
        fgets(str, sizeof(str)-1, fp);

        /* Search for comment and cut it off */
        s = strchr(str, '#');
        if(NULL != s) {
            *s = '\0'; 
        }

        if(0 == strlen(str)) {
            continue;
        }

        /* Skip leading spaces */
        for(s=str; isspace(*s); s++);

        /* Skip trailing spaces */
        for(i=strlen(s)-1; i > 0 && isspace(s[i]); i--);
        s[i+1] = '\0';

        if(0 != strlen(s)) {
            process_config_line(s, line);
        }
    }

    fclose(fp);

    /* Copy modified data back */
    memcpy(opts, &_opts, sizeof(*opts));

    return 0;
}